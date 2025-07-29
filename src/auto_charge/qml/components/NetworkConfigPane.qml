import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    property string ip: "127.0.0.1"
    property int port: 59999
    property int connectionTimeout: 30
    signal configChanged(string ip, int port, int timeout)

    spacing: 16

    Label {
        text: "网络配置"
        font.pixelSize: 18
        font.weight: Font.Medium
    }

    // IP地址配置
    RowLayout {
        Layout.fillWidth: true

        Label {
            text: "IP地址"
            Layout.preferredWidth: 80
        }

        TextField {
            id: ipTextField
            text: ip
            placeholderText: "请输入IP地址"
            Layout.fillWidth: true
            inputMethodHints: Qt.ImhDigitsOnly

            validator: RegularExpressionValidator {
                regularExpression: /^(\d{1,3}\.){3}\d{1,3}$/
            }

            onTextChanged: {
                if (text.length > 0) {
                    let cleaned = text.replace(/[^\d.]/g, '');
                    let parts = cleaned.split('.');
                    for (let i = 0; i < parts.length; i++) {
                        if (parts[i].length > 3) {
                            parts[i] = parts[i].substring(0, 3);
                        }
                    }
                    let result = parts.join('.');
                    if (result !== text) {
                        ipTextField.text = result;
                        ipTextField.cursorPosition = result.length;
                    }
                }
                ip = ipTextField.text;
                configChanged(ip, port, connectionTimeout);
            }

            Keys.onPressed: function (event) {
                if (event.key === Qt.Key_Period || event.key === Qt.Key_Comma) {
                    event.accepted = true;
                    let pos = cursorPosition;
                    let newText = text.substring(0, pos) + "." + text.substring(pos);
                    text = newText;
                    cursorPosition = pos + 1;
                }
            }
        }
    }

    // 端口配置
    RowLayout {
        Layout.fillWidth: true

        Label {
            text: "端口"
            Layout.preferredWidth: 80
        }

        SpinBox {
            id: portSpinBox
            from: 1
            to: 65535
            value: port
            Layout.fillWidth: true

            onValueChanged: {
                port = value;
                configChanged(ip, port, connectionTimeout);
            }
        }
    }

    // 连接超时
    RowLayout {
        Layout.fillWidth: true

        Label {
            text: "连接超时"
            Layout.preferredWidth: 80
        }

        SpinBox {
            id: connectionTimeoutSpinBox
            from: 1
            to: 60
            value: connectionTimeout
            Layout.fillWidth: true

            onValueChanged: {
                connectionTimeout = value;
                configChanged(ip, port, connectionTimeout);
            }
        }
    }
}
