/**
 * @file px4_msgs_cache.hpp
 * @brief PX4消息缓存类的头文件
 * @author WWM
 * @date 2025-09-14
 * @version 1.0
 */

#pragma once

#include <mutex>
#include <utility>

#include "px4_comm_types.hpp"

template <typename T>
class ThreadSafeCache {
 private:
  T data_;
  mutable std::mutex mutex_;

 public:
  ThreadSafeCache() = default;
  explicit ThreadSafeCache(T initial_data) : data_(std::move(initial_data)) {}

  ThreadSafeCache(const ThreadSafeCache &) = delete;
  ThreadSafeCache &operator=(const ThreadSafeCache &) = delete;

  void update(const T &new_data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = new_data;
  }
  T get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  template <typename F>
  auto access(F &&func) const -> decltype(func(data_)) {
    std::lock_guard<std::mutex> lock(mutex_);
    return func(data_);
  }
};

/**
 * @class Px4MsgsCache
 * @brief PX4消息缓存类
 *
 * 该类用于缓存和管理PX4飞控系统的各种消息数据，包括位置、飞行状态和电池状态等。
 * 提供线程安全的更新和读取接口，通过ThreadSafeCache模板保证多线程环境下的数据一致性。
 *
 * @details
 * 主要功能包括：
 * - 缓存PX4的NED坐标系位置信息
 * - 缓存PX4的电池状态信息
 * - 缓存PX4的飞行器状态信息
 * - 提供线程安全的数据访问接口
 *
 * @note 所有的数据访问操作都是线程安全的
 * @warning 在高频率访问时需要注意性能影响
 */
class Px4MsgsCache {
 private:
  /**
   * @brief PX4的NED坐标系位置数据缓存
   * @details
   * 使用ThreadSafeCache模板存储无人机在North-East-Down坐标系中的位置信息
   */
  ThreadSafeCache<px4Status::VehiclePose> vehicle_pose_cache_;

  /**
   * @brief PX4的电池状态数据缓存
   * @details 使用ThreadSafeCache模板存储电池电压、电流、剩余电量等信息
   */
  ThreadSafeCache<px4Status::BatteryStatus> battery_status_cache_;

  /**
   * @brief PX4的飞行器状态数据缓存
   * @details 使用ThreadSafeCache模板存储飞行模式、解锁状态、导航状态等信息
   */
  ThreadSafeCache<px4Status::VehicleStatus> vehicle_status_cache_;

 public:
  /**
   * @brief 默认构造函数
   * @details 初始化Px4MsgsCache对象，所有成员变量使用默认值
   */
  Px4MsgsCache() = default;

  /**
   * @brief 更新NED坐标系位置数据
   * @param new_position 新的位置数据
   * @details 线程安全地更新位置缓存
   * @note 该操作通过ThreadSafeCache模板自动保证线程安全
   */
  void updateVehiclePose(const px4Status::VehiclePose &new_position);

  /**
   * @brief 更新电池状态数据
   * @param new_status 新的电池状态数据
   * @details 线程安全地更新电池状态缓存
   * @note 该操作通过ThreadSafeCache模板自动保证线程安全
   */
  void updateBatteryStatus(const px4Status::BatteryStatus &new_status);

  /**
   * @brief 更新飞行器状态数据
   * @param new_status 新的飞行器状态数据
   * @details 线程安全地更新飞行器状态缓存
   * @note 该操作通过ThreadSafeCache模板自动保证线程安全
   */
  void updateVehicleStatus(const px4Status::VehicleStatus &new_status);

  /**
   * @brief 获取当前NED坐标系位置数据
   * @return 当前的位置数据副本
   * @details 线程安全地读取位置缓存
   * @note 该操作通过ThreadSafeCache模板自动保证线程安全，返回数据的副本
   */
  px4Status::VehiclePose getVehiclePose() const;

  /**
   * @brief 获取当前电池状态数据
   * @return px4Status::BatteryStatus 当前的电池状态数据副本
   * @details 线程安全地读取电池状态缓存
   * @note 该操作通过ThreadSafeCache模板自动保证线程安全，返回数据的副本
   */
  px4Status::BatteryStatus getBatteryStatus() const;

  /**
   * @brief 获取当前飞行器状态数据
   * @return px4Status::VehicleStatus 当前的飞行器状态数据副本
   * @details 线程安全地读取飞行器状态缓存
   * @note 该操作通过ThreadSafeCache模板自动保证线程安全，返回数据的副本
   */
  px4Status::VehicleStatus getVehicleStatus() const;

  /**
   * @brief 使用函数对象访问位置数据
   * @tparam F 函数对象类型
   * @param func 访问函数
   * @return 函数的返回值
   * @details 提供线程安全的函数式访问接口，允许在持有锁的情况下对数据进行操作
   */
  template <typename F>
  auto accessVehiclePose(F &&func) const
      -> decltype(func(std::declval<px4Status::VehiclePose>())) {
    return vehicle_pose_cache_.access(std::forward<F>(func));
  }

  /**
   * @brief 使用函数对象访问电池状态数据
   * @tparam F 函数对象类型
   * @param func 访问函数
   * @return 函数的返回值
   * @details 提供线程安全的函数式访问接口，允许在持有锁的情况下对数据进行操作
   */
  template <typename F>
  auto accessBatteryStatus(F &&func) const
      -> decltype(func(std::declval<px4Status::BatteryStatus>())) {
    return battery_status_cache_.access(std::forward<F>(func));
  }

  /**
   * @brief 使用函数对象访问飞行器状态数据
   * @tparam F 函数对象类型
   * @param func 访问函数
   * @return 函数的返回值
   * @details 提供线程安全的函数式访问接口，允许在持有锁的情况下对数据进行操作
   */
  template <typename F>
  auto accessVehicleStatus(F &&func) const
      -> decltype(func(std::declval<px4Status::VehicleStatus>())) {
    return vehicle_status_cache_.access(std::forward<F>(func));
  }
};
