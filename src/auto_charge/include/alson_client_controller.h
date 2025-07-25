// alson_client_controller.h
#include <QObject>
#include <jaka_msgs/srv/update_camera_para.hpp>
#include <rclcpp/rclcpp.hpp>

class AlsonClientController : public QObject {
  Q_OBJECT

public:
  explicit AlsonClientController(QObject *parent = nullptr);

  Q_INVOKABLE void disconnectClient();
  Q_INVOKABLE void reconnectClient();
  Q_INVOKABLE void updateClientParam(const QString &key, const QString &value);

signals:
  void clientStatusChanged(QString status);
  void clientEvent(QString eventType, QVariantMap eventData);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<jaka_msgs::srv::UpdateCameraPara>::SharedPtr
      update_camera_para_client_;
  // ... 其它成员
};
