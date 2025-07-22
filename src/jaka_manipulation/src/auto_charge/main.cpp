#include <QApplication>
#include <QLabel>
#include <QMainWindow>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  QMainWindow window;
  window.setWindowTitle("Auto Charge GUI");
  window.resize(400, 300);
  QLabel *label = new QLabel("Hello, Qt6!", &window);
  label->setAlignment(Qt::AlignCenter);
  window.setCentralWidget(label);
  window.show();
  return app.exec();
}