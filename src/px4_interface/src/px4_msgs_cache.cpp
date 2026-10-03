#include "px4_interface/px4_msgs_cache.hpp"

void Px4MsgsCache::updateVehiclePose(
    const px4Status::VehiclePose& new_position) {
  vehicle_pose_cache_.update(new_position);
}

void Px4MsgsCache::updateBatteryStatus(
    const px4Status::BatteryStatus& new_status) {
  battery_status_cache_.update(new_status);
}

void Px4MsgsCache::updateVehicleStatus(
    const px4Status::VehicleStatus& new_status) {
  vehicle_status_cache_.update(new_status);
}

px4Status::VehiclePose Px4MsgsCache::getVehiclePose() const {
  return vehicle_pose_cache_.get();
}

px4Status::BatteryStatus Px4MsgsCache::getBatteryStatus() const {
  return battery_status_cache_.get();
}

px4Status::VehicleStatus Px4MsgsCache::getVehicleStatus() const {
  return vehicle_status_cache_.get();
}
