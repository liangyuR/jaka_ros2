#include "auto_charge/auto_charge_node.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace Qt::StringLiterals;

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  // 设置应用信息
  QGuiApplication::setApplicationName("Auto Charge Manager");
  QGuiApplication::setApplicationVersion("0.1.0");
  QGuiApplication::setOrganizationName("Auto Charge Team");

  // 设置Material样式
  QQuickStyle::setStyle("Material");

  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建AutoChargeNode实例
  auto auto_charge_node = std::make_shared<auto_charge::AutoChargeNode>();

  QQmlApplicationEngine engine;

  // 将CameraController直接注册到QML上下文
  engine.rootContext()->setContextProperty(
      "cameraController", auto_charge_node->GetCameraController());

  // 加载主QML文件
  const QUrl url(u"qrc:/main.qml"_s);
  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreated, &app,
      [url](QObject *obj, const QUrl &objUrl) {
        if (obj == nullptr && url == objUrl)
          QCoreApplication::exit(-1);
      },
      Qt::QueuedConnection);

  engine.load(url);
  std::thread ros_thread([&]() { rclcpp::spin(auto_charge_node); });
  int result = QGuiApplication::exec();

  // 清理ROS2
  rclcpp::shutdown();

  return result;
}