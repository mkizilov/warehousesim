#include "warehousesim/tasks/task.hpp"

namespace warehousesim::tasks {

const char* toString(TaskPriority priority) noexcept {
  switch (priority) {
    case TaskPriority::Low:
      return "low";
    case TaskPriority::Normal:
      return "normal";
    case TaskPriority::High:
      return "high";
    case TaskPriority::Urgent:
      return "urgent";
  }
  return "unknown";
}

const char* toString(TaskStatus status) noexcept {
  switch (status) {
    case TaskStatus::Pending:
      return "pending";
    case TaskStatus::Assigned:
      return "assigned";
    case TaskStatus::InProgress:
      return "in_progress";
    case TaskStatus::Completed:
      return "completed";
    case TaskStatus::Failed:
      return "failed";
    case TaskStatus::Cancelled:
      return "cancelled";
  }
  return "unknown";
}

}  // namespace warehousesim::tasks
