#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  // 设置应用信息
  app.setApplicationName("Auto Charge Manager");
  app.setApplicationVersion("0.1.0");
  app.setOrganizationName("Auto Charge Team");

  // 设置Material样式
  QQuickStyle::setStyle("Material");

  QQmlApplicationEngine engine;

  // 加载主QML文件
  const QUrl url(u"qrc:/main.qml"_qs);
  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreated, &app,
      [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
          QCoreApplication::exit(-1);
      },
      Qt::QueuedConnection);

  engine.load(url);

  return app.exec();
}