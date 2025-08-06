import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import "../components"

// 基础页面组件，提供通用的页面布局结构
Item {
    property string pageTitle: "基础设置"

    Component.onCompleted: {
        robotManager.delayInit()
    }

    clip: true

    ScrollView {
        id: scrollView
        anchors.fill: parent
        clip: true

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 24

            // 页面标题
            Label {
                text: pageTitle
                font.pixelSize: 28
                font.weight: Font.Medium
                Layout.topMargin: 24
                Layout.leftMargin: 24
            }

            // 机械臂连接配置卡片
            Pane {
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                Layout.preferredWidth: scrollView.width - 48

                ColumnLayout {
                    Layout.alignment: Qt.AlignLeft
                    Layout.fillWidth: true
                    spacing: 12

                    RowLayout {
                        spacing: 8
                        IPTextField {
                            id: ipField
                            ipAddress: robotManager.ip
                            Layout.preferredWidth: 180
                            Layout.alignment: Qt.AlignVCenter
                        }

                        Button {
                            text: "更新配置"
                            Layout.alignment: Qt.AlignVCenter
                            Material.foreground: "white"
                            Material.background: Material.primary
                            onClicked: {
                                robotManager.ip = ipField.text
                                robotManager.updateRobotConnectConfig()
                            }
                        }

                        Item { width: 24 } // 间隔

                        RotatingButton {
                            id: connectButton
                            text: robotManager.connected ? "已连接" : "未连接"
                            enabled: !robotManager.connecting
                            Material.background: robotManager.connected ? Material.Green : Material.Red
                            Material.foreground: "white"
                            isRotating: robotManager.connecting
                            
                            onClicked: {
                                if (robotManager.connected) {
                                    robotManager.disconnect()
                                } else {
                                    robotManager.connect()
                                }
                            }
                        }
                    }
                }
            }

            // 基础控制卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                Material.foreground: Material.White

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 16

                    Label {
                        text: "基础控制"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                        Material.foreground: Material.Black
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 32
                        Layout.alignment: Qt.AlignLeft

                        // 电源控制按钮
                        RotatingButton {
                            text: robotManager.poweredOn ? "已上电" : "未上电"
                            enabled: robotManager.connected && !robotManager.connecting && !robotManager.powering
                            Material.background: robotManager.poweredOn ? Material.Green : Material.Red
                            Material.foreground: "white"
                            isRotating: robotManager.powering
                            onClicked: {
                                robotManager.power(!robotManager.poweredOn)
                            }
                        }

                        // 使能控制按钮
                        RotatingButton {
                            text: robotManager.enabled ? "已使能" : "未使能"
                            enabled: robotManager.connected && robotManager.poweredOn && !robotManager.connecting && !robotManager.enabling
                            Material.background: robotManager.enabled ? Material.Green : Material.Red
                            Material.foreground: "white"
                            isRotating: robotManager.enabling
                            onClicked: {
                                robotManager.enable(!robotManager.enabled)
                            }
                        }

                        // 错误状态按钮
                        Button {
                            text: robotManager.error ? "错误" : "正常"
                            enabled: robotManager.connected && robotManager.error && !robotManager.connecting
                            Material.background: robotManager.error ? Material.Orange : Material.Grey
                            onClicked: {
                                robotManager.clearError()
                            }
                        }
                    }
                }
            }
        }
    }
}