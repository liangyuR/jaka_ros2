#ifndef JAKA_YAML_CONVERTER_H
#define JAKA_YAML_CONVERTER_H

#include "jaka_sdk/jktypes.h"
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

// 为 jktypes.h 中的结构体添加 YAML 转换功能

// CartesianTran 转换器
inline YAML::Node toYaml(const CartesianTran &tran) {
  YAML::Node node;
  node["x"] = tran.x;
  node["y"] = tran.y;
  node["z"] = tran.z;
  return node;
}

inline void fromYaml(const YAML::Node &node, CartesianTran &tran) {
  if (node["x"])
    tran.x = node["x"].as<double>();
  if (node["y"])
    tran.y = node["y"].as<double>();
  if (node["z"])
    tran.z = node["z"].as<double>();
}

// Rpy 转换器
inline YAML::Node toYaml(const Rpy &rpy) {
  YAML::Node node;
  node["rx"] = rpy.rx;
  node["ry"] = rpy.ry;
  node["rz"] = rpy.rz;
  return node;
}

inline void fromYaml(const YAML::Node &node, Rpy &rpy) {
  if (node["rx"])
    rpy.rx = node["rx"].as<double>();
  if (node["ry"])
    rpy.ry = node["ry"].as<double>();
  if (node["rz"])
    rpy.rz = node["rz"].as<double>();
}

// CartesianPose 转换器
inline YAML::Node toYaml(const CartesianPose &pose) {
  YAML::Node node;
  node["tran"] = toYaml(pose.tran);
  node["rpy"] = toYaml(pose.rpy);
  return node;
}

inline void fromYaml(const YAML::Node &node, CartesianPose &pose) {
  if (node["tran"])
    fromYaml(node["tran"], pose.tran);
  if (node["rpy"])
    fromYaml(node["rpy"], pose.rpy);
}

// JointValue 转换器
inline YAML::Node toYaml(const JointValue &joints) {
  YAML::Node node;
  YAML::Node jvals;
  for (int i = 0; i < 6; ++i) {
    jvals.push_back(joints.jVal[i]);
  }
  node["jVal"] = jvals;
  return node;
}

inline void fromYaml(const YAML::Node &node, JointValue &joints) {
  if (node["jVal"]) {
    auto jvals = node["jVal"];
    for (int i = 0; i < 6 && i < jvals.size(); ++i) {
      joints.jVal[i] = jvals[i].as<double>();
    }
  }
}

// AdmitCtrlType 转换器 (你已经有的结构体)
inline YAML::Node toYaml(const AdmitCtrlType &admit) {
  YAML::Node node;
  node["axis"] = admit.axis;
  node["opt"] = admit.opt;
  node["ft_user"] = admit.ft_user;
  node["ft_rebound"] = admit.ft_rebound;
  node["ft_constant"] = admit.ft_constant;
  node["ft_normal_track"] = admit.ft_normal_track;
  return node;
}

inline void fromYaml(const YAML::Node &node, AdmitCtrlType &admit) {
  if (node["axis"])
    admit.axis = node["axis"].as<int>();
  if (node["opt"])
    admit.opt = node["opt"].as<int>();
  if (node["ft_user"])
    admit.ft_user = node["ft_user"].as<double>();
  if (node["ft_rebound"])
    admit.ft_rebound = node["ft_rebound"].as<double>();
  if (node["ft_constant"])
    admit.ft_constant = node["ft_constant"].as<double>();
  if (node["ft_normal_track"])
    admit.ft_normal_track = node["ft_normal_track"].as<int>();
}

// AdmitCtrlTypeList 转换器
inline YAML::Node toYaml(const AdmitCtrlTypeList &list) {
  YAML::Node node;
  YAML::Node admit_list;
  for (const auto &admit : list.admit_ctrl) {
    admit_list.push_back(toYaml(admit));
  }
  node["admit_ctrl"] = admit_list;
  return node;
}

inline void fromYaml(const YAML::Node &node, AdmitCtrlTypeList &list) {
  if (node["admit_ctrl"]) {
    list.admit_ctrl.clear();
    auto admit_list = node["admit_ctrl"];
    for (const auto &admit_node : admit_list) {
      AdmitCtrlType admit;
      fromYaml(admit_node, admit);
      list.admit_ctrl.push_back(admit);
    }
  }
}

// 工具函数
namespace JakaYamlUtils {
// 保存结构体到文件
template <typename T>
bool saveToFile(const T &obj, const std::string &filename) {
  try {
    YAML::Node node = toYaml(obj);
    std::ofstream file(filename);
    file << node;
    return true;
  } catch (const std::exception &e) {
    std::cerr << "保存失败: " << e.what() << std::endl;
    return false;
  }
}

// 从文件加载结构体
template <typename T> bool loadFromFile(const std::string &filename, T &obj) {
  try {
    YAML::Node node = YAML::LoadFile(filename);
    fromYaml(node, obj);
    return true;
  } catch (const std::exception &e) {
    std::cerr << "加载失败: " << e.what() << std::endl;
    return false;
  }
}

// 结构体转 YAML 字符串
template <typename T> std::string structToYamlString(const T &obj) {
  YAML::Node node = toYaml(obj);
  YAML::Emitter emitter;
  emitter << node;
  return emitter.c_str();
}

// YAML 字符串转结构体
template <typename T>
bool yamlStringToStruct(const std::string &yaml_str, T &obj) {
  try {
    YAML::Node node = YAML::Load(yaml_str);
    fromYaml(node, obj);
    return true;
  } catch (const std::exception &e) {
    std::cerr << "解析失败: " << e.what() << std::endl;
    return false;
  }
}
} // namespace JakaYamlUtils

#endif // JAKA_YAML_CONVERTER_H