#include "warehousesim/rendering/sdl_window.hpp"

#include <SDL3/SDL.h>

#include <optional>
#include <stdexcept>
#include <variant>

namespace warehousesim::rendering {

using view::Color;
using view::DrawCommand;
using view::DrawList;
using view::FillRect;
using view::InputEvents;
using view::Line;
using view::ScreenPoint;
using view::ScreenRect;
using view::ScreenSize;
using view::Text;

namespace {

[[noreturn]] void throwSdlError(const char* what) {
  throw std::runtime_error(std::string(what) + ": " + SDL_GetError());
}

SDL_FRect toSdl(const ScreenRect& rect) {
  return {static_cast<float>(rect.x), static_cast<float>(rect.y), static_cast<float>(rect.width),
          static_cast<float>(rect.height)};
}

bool isMouseEvent(Uint32 type) {
  return type == SDL_EVENT_MOUSE_MOTION || type == SDL_EVENT_MOUSE_WHEEL ||
         type == SDL_EVENT_MOUSE_BUTTON_DOWN || type == SDL_EVENT_MOUSE_BUTTON_UP;
}

std::optional<view::PointerButton> pointerButtonOf(Uint8 button) {
  switch (button) {
    case SDL_BUTTON_LEFT:
      return view::PointerButton::Primary;
    case SDL_BUTTON_RIGHT:
      return view::PointerButton::Secondary;
    default:
      return std::nullopt;
  }
}

// What a key press means to the map editor, if anything. Control (Command on a Mac) with a letter
// is a file or history command, Shift with an arrow resizes the map, the rest are single keys.
std::optional<view::EditorCommand> editorCommandFor(SDL_Keycode key, SDL_Keymod mod, bool repeat) {
  using view::EditorCommand;
  if ((mod & (SDL_KMOD_CTRL | SDL_KMOD_GUI)) != 0) {
    const bool shift = (mod & SDL_KMOD_SHIFT) != 0;
    switch (key) {
      case SDLK_Z:
        return shift ? EditorCommand::Redo : EditorCommand::Undo;
      case SDLK_Y:
        return EditorCommand::Redo;
      case SDLK_S:
        return repeat ? std::nullopt : std::optional(EditorCommand::Save);
      case SDLK_O:
        return repeat ? std::nullopt : std::optional(EditorCommand::Load);
      default:
        return std::nullopt;
    }
  }
  if ((mod & SDL_KMOD_SHIFT) != 0) {
    switch (key) {
      case SDLK_RIGHT:
        return EditorCommand::Wider;
      case SDLK_LEFT:
        return EditorCommand::Narrower;
      case SDLK_DOWN:
        return EditorCommand::Taller;
      case SDLK_UP:
        return EditorCommand::Shorter;
      default:
        return std::nullopt;
    }
  }
  if (repeat) {
    return std::nullopt;
  }
  switch (key) {
    case SDLK_1:
      return EditorCommand::ToolWall;
    case SDLK_2:
      return EditorCommand::ToolShelf;
    case SDLK_3:
      return EditorCommand::ToolPickup;
    case SDLK_4:
      return EditorCommand::ToolDropoff;
    case SDLK_5:
      return EditorCommand::ToolCharging;
    case SDLK_6:
      return EditorCommand::ToolErase;
    case SDLK_L:
      return EditorCommand::NextLayout;
    case SDLK_DELETE:
    case SDLK_BACKSPACE:
      return EditorCommand::Clear;
    default:
      return std::nullopt;
  }
}

}  // namespace

struct SdlWindow::Impl {
  struct Session {
    Session() {
      if (!SDL_Init(SDL_INIT_VIDEO)) {
        throwSdlError("SDL_Init failed");
      }
    }
    ~Session() { SDL_Quit(); }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
  };

  // Declaration order is destruction order in reverse: renderer, window, then SDL itself.
  Session session;
  std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window{nullptr, &SDL_DestroyWindow};
  std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer{nullptr,
                                                                         &SDL_DestroyRenderer};

  void setColor(Color color) const {
    if (!SDL_SetRenderDrawColor(renderer.get(), color.r, color.g, color.b, color.a)) {
      throwSdlError("SDL_SetRenderDrawColor failed");
    }
  }

  void execute(const FillRect& command) const {
    setColor(command.color);
    const SDL_FRect rect = toSdl(command.rect);
    if (!SDL_RenderFillRect(renderer.get(), &rect)) {
      throwSdlError("SDL_RenderFillRect failed");
    }
  }

  void execute(const Line& command) const {
    setColor(command.color);
    if (!SDL_RenderLine(renderer.get(), static_cast<float>(command.from.x),
                        static_cast<float>(command.from.y), static_cast<float>(command.to.x),
                        static_cast<float>(command.to.y))) {
      throwSdlError("SDL_RenderLine failed");
    }
  }

  void execute(const Text& command) const {
    setColor(command.color);
    if (!SDL_RenderDebugText(renderer.get(), static_cast<float>(command.position.x),
                             static_cast<float>(command.position.y), command.text.c_str())) {
      throwSdlError("SDL_RenderDebugText failed");
    }
  }
};

SdlWindow::SdlWindow(const std::string& title, int width, int height)
    : impl_(std::make_unique<Impl>()) {
  impl_->window.reset(SDL_CreateWindow(title.c_str(), width, height, SDL_WINDOW_RESIZABLE));
  if (!impl_->window) {
    throwSdlError("SDL_CreateWindow failed");
  }
  impl_->renderer.reset(SDL_CreateRenderer(impl_->window.get(), nullptr));
  if (!impl_->renderer) {
    throwSdlError("SDL_CreateRenderer failed");
  }
}

SdlWindow::~SdlWindow() = default;

InputEvents SdlWindow::pollEvents(PollMode mode) {
  const bool editing = mode == PollMode::Editor;
  InputEvents result;
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    // Mouse positions arrive in window units; the camera works in the pixels we draw in.
    if (isMouseEvent(event.type) &&
        !SDL_ConvertEventToRenderCoordinates(impl_->renderer.get(), &event)) {
      throwSdlError("SDL_ConvertEventToRenderCoordinates failed");
    }

    switch (event.type) {
      case SDL_EVENT_QUIT:
        result.quit = true;
        break;
      case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        result.pointer.reset();
        result.pointerLeft = true;
        break;
      case SDL_EVENT_MOUSE_MOTION: {
        result.pointer = ScreenPoint{event.motion.x, event.motion.y};
        result.pointerLeft = false;
        // The simulation view pans with any button; the editor paints with two and pans with one.
        const Uint32 dragButtons =
            editing ? SDL_BUTTON_MMASK : (SDL_BUTTON_LMASK | SDL_BUTTON_MMASK | SDL_BUTTON_RMASK);
        if ((event.motion.state & dragButtons) != 0) {
          result.dragX += event.motion.xrel;
          result.dragY += event.motion.yrel;
        }
        if (editing && (event.motion.state & (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK)) != 0) {
          result.pointerActions.push_back({view::PointerAction::Kind::Move,
                                           view::PointerButton::Primary,
                                           ScreenPoint{event.motion.x, event.motion.y}});
        }
        break;
      }
      case SDL_EVENT_MOUSE_BUTTON_DOWN:
      case SDL_EVENT_MOUSE_BUTTON_UP:
        if (editing) {
          if (const auto button = pointerButtonOf(event.button.button)) {
            const bool pressed = event.type == SDL_EVENT_MOUSE_BUTTON_DOWN;
            result.pointerActions.push_back(
                {pressed ? view::PointerAction::Kind::Press : view::PointerAction::Kind::Release,
                 *button, ScreenPoint{event.button.x, event.button.y}});
            result.pointer = ScreenPoint{event.button.x, event.button.y};
            result.pointerLeft = false;
          }
        }
        break;
      case SDL_EVENT_MOUSE_WHEEL: {
        const float direction = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -1.0F : 1.0F;
        result.zoomSteps += static_cast<double>(event.wheel.y * direction);
        result.zoomAnchor = ScreenPoint{event.wheel.mouse_x, event.wheel.mouse_y};
        result.pointer = result.zoomAnchor;
        result.pointerLeft = false;
        break;
      }
      case SDL_EVENT_KEY_DOWN: {
        const SDL_Keycode key = event.key.key;
        if (editing) {
          if (const auto command = editorCommandFor(key, event.key.mod, event.key.repeat)) {
            result.commands.push_back(*command);
            break;
          }
        }
        if (key == SDLK_EQUALS || key == SDLK_PLUS || key == SDLK_KP_PLUS) {
          result.zoomSteps += 1.0;  // auto-repeat gives smooth zooming while held
        } else if (key == SDLK_MINUS || key == SDLK_KP_MINUS) {
          result.zoomSteps -= 1.0;
        } else if (event.key.repeat) {
          break;
        } else if (key == SDLK_ESCAPE) {
          result.quit = true;
        } else if (key == SDLK_SPACE) {
          result.togglePause = !result.togglePause;
        } else if (key == SDLK_0 || key == SDLK_KP_0) {
          result.resetView = true;
        } else if (key == SDLK_P) {
          result.togglePaths = !result.togglePaths;
        } else if (key == SDLK_O) {
          result.toggleOccupied = !result.toggleOccupied;
        } else if (key == SDLK_LEFTBRACKET) {
          --result.speedSteps;
        } else if (key == SDLK_RIGHTBRACKET) {
          ++result.speedSteps;
        }
        break;
      }
      default:
        break;
    }
  }

  const bool* keys = SDL_GetKeyboardState(nullptr);
  const auto held = [keys](SDL_Scancode a, SDL_Scancode b) { return keys[a] || keys[b]; };
  result.panX = (held(SDL_SCANCODE_RIGHT, SDL_SCANCODE_D) ? 1.0 : 0.0) -
                (held(SDL_SCANCODE_LEFT, SDL_SCANCODE_A) ? 1.0 : 0.0);
  result.panY = (held(SDL_SCANCODE_DOWN, SDL_SCANCODE_S) ? 1.0 : 0.0) -
                (held(SDL_SCANCODE_UP, SDL_SCANCODE_W) ? 1.0 : 0.0);
  if (editing && (SDL_GetModState() & (SDL_KMOD_SHIFT | SDL_KMOD_CTRL | SDL_KMOD_GUI)) != 0) {
    // Shift+arrow resizes the map and Ctrl+S saves it: neither should also move the view.
    result.panX = 0.0;
    result.panY = 0.0;
  }
  return result;
}

ScreenSize SdlWindow::renderSize() const {
  int width = 0;
  int height = 0;
  if (!SDL_GetCurrentRenderOutputSize(impl_->renderer.get(), &width, &height)) {
    throwSdlError("SDL_GetCurrentRenderOutputSize failed");
  }
  return {width, height};
}

void SdlWindow::draw(const DrawList& list) {
  impl_->setColor(list.background);
  if (!SDL_RenderClear(impl_->renderer.get())) {
    throwSdlError("SDL_RenderClear failed");
  }
  for (const DrawCommand& command : list.commands) {
    std::visit([this](const auto& typed) { impl_->execute(typed); }, command);
  }
  if (!SDL_RenderPresent(impl_->renderer.get())) {
    throwSdlError("SDL_RenderPresent failed");
  }
}

}  // namespace warehousesim::rendering
