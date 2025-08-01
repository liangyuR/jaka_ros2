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

  QQuickStyle::setStyle("Material");

  rclcpp::init(argc, argv);

  auto auto_charge_node = std::make_shared<auto_charge::AutoChargeNode>();

  QQmlApplicationEngine engine;
  // clang-format off
  engine.rootContext()->setContextProperty("cameraController", auto_charge_node->GetCameraController());
  engine.rootContext()->setContextProperty("robotManager", auto_charge_node->GetRobotManager());
  // clang-format on

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

  // 创建多线程执行器来管理多个节点
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(auto_charge_node);
  executor->add_node(auto_charge_node->GetCameraController()->getNode());
  executor->add_node(auto_charge_node->GetRobotManager()->getNode());

  std::thread ros_thread([&]() { executor->spin(); });
  int result = QGuiApplication::exec();

  rclcpp::shutdown();

  return result;
}