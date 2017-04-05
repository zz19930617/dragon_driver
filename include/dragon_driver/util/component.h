/*
 * component.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_UTIL_COMPONENT_H_
#define INCLUDE_MIDDLEWARE_UTIL_COMPONENT_H_

#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include "log.h"

namespace middleware {
/**
 * 完成组合模式的基类.
 * 组合模式以名称为标识符
 */
template <typename T>
class Component {
public:
  typedef typename std::map<std::string, boost::shared_ptr<T>>::iterator iterator;
  /**************************************************
   * 下述两个函数完成component的新增与删除
   * 以名称为标识符
   **************************************************/
  void add(const std::string& name, T* component) {
    boost::shared_ptr<T> ptr(component);
    add(name, ptr);
  }

  void add(const std::string& name, boost::shared_ptr<T> component) {
    auto itr = composite_.find(name);
    if (composite_.end() == itr) {
      // LOG_INFO << "Addition component( " << name << " )";
      composite_.insert(std::make_pair(name, component));
    } else {
      LOG(WARNING) << "Replace component( " << name << " )";
      itr->second.swap(component);
    }
  }

  void remove(T* component) {
    for (auto itr : composite_) {
      if (itr->second.get() == component) {
        // LOG_INFO << "Remove component( " << itr->first << " )";
        composite_.erase(itr);
        return;
      }
    }
    LOG(WARNING) << "Can't found the handle by addr: (" << component << " )";
  }

  void remove(boost::shared_ptr<T> component) {
    remove(component.get());
  }

  void remove(const std::string& name) {
    auto itr = composite_.find(name);
    if (composite_.end() != itr) {
      composite_.erase(itr);
    }
    LOG_INFO << "Remove component( " << name << " )";
  }

  /**************************************************
   * 查询是否包含某个名称的零件, 并返回迭代器, 不存在, 则返回end()
   * 并定义begin(), end()及一些常用操作
   **************************************************/
  iterator find(const std::string& name) { return composite_.find(name); }
  iterator begin() { return composite_.begin(); }
  iterator end() { return composite_.end(); }
  boost::shared_ptr<T>& operator[](const std::string& key) { return composite_[key]; }

  /*
  template <typename Type>
  friend bool operator==(const Component<Type>&, const Component<Type>&);

  template <typename Type>
  friend bool operator<(const Component<Type>&, const Component<Type>&);
  */

protected:
  std::map<std::string, boost::shared_ptr<T>> composite_;
};
/*
template <typename T>
bool operator!=(const Component<T>&, const Component<T>&);

template <typename T>
bool operator>=(const Component<T>&, const Component<T>&);

template <typename T>
bool operator>(const Component<T>&, const Component<T>&);

template <typename T>
bool operator<=(const Component<T>&, const Component<T>&);
*/
} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_UTIL_COMPONENT_H_ */
