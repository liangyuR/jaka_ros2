import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import "./components"

Item {
    id: robot_control_ui
    
    // 信号定义
    signal connectRobot(string ip, int port)
    signal disconnectRobot()
    signal enableRobot()
    signal disableRobot()
    signal clearError()
    signal resetSensor()
    
    // 属性
    property string currentIp: "127.0.0.1"
    property int currentPort: 8080
    property bool isConnected: false
    property bool isConnecting: false
    property bool isEnabled: false
    property bool hasError: false
    
    // 主布局
    ScrollView {
        anchors.fill: parent
        clip: true
        
        ColumnLayout {
            width: parent.width
            spacing: 24
            
            // 页面标题
            Label {
                text: "机械臂控制"
                font.pixelSize: 28
                font.weight: Font.Medium
                Layout.topMargin: 24
                Layout.leftMargin: 24
            }
            
            // 连接状态卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                
                RowLayout {
                    anchors.fill: parent
                    spacing: 16
                    
                    // 状态图标
                    Rectangle {
                        width: 16
                        height: 16
                        radius: 8
                        color: isConnected ? Material.Green : Material.Red
                        
                        BusyIndicator {
                            anchors.centerIn: parent
                            width: 12
                            height: 12
                            running: isConnecting
                            visible: isConnecting
                        }
                    }
                    
                    // 状态文本
                    Label {
                        text: {
                            if (isConnecting) return "连接中..."
                            else if (isConnected) return "已连接"
                            else return "未连接"
                        }
                        font.pixelSize: 16
                        font.weight: Font.Medium
                    }
                    
                    Item { Layout.fillWidth: true }
                    
                    // 使能状态
                    Label {
                        text: isEnabled ? "已使能" : "未使能"
                        font.pixelSize: 12
                        color: isEnabled ? Material.Green : Material.Grey
                        visible: isConnected
                    }
                    
                    // 错误状态
                    Label {
                        text: "有错误"
                        font.pixelSize: 12
                        color: Material.Red
                        visible: isConnected && hasError
                    }
                }
            }
            
            // 网络配置卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1

                NetworkConfigPane {
                    id: netConfig
                    ip: currentIp
                    port: currentPort
                    onConfigChanged: function(ip, port) {
                        currentIp = ip
                        currentPort = port
                    }
                }
            }
            
            // 连接控制卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                
                RowLayout {
                    anchors.fill: parent
                    spacing: 16
                    
                    Label {
                        text: "连接控制"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                    }
                    
                    Item { Layout.fillWidth: true }
                    
                    // 连接/断开按钮
                    Button {
                        text: isConnected ? "断开连接" : "连接"
                        enabled: !isConnecting
                        Material.background: isConnected ? Material.Red : Material.Green
                        Material.foreground: "white"
                        onClicked: {
                            if (isConnected) {
                                disconnectRobot()
                            } else {
                                connectRobot(currentIp, currentPort)
                            }
                        }
                    }
                }
            }
            
            // 操作按钮卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1

                RowLayout {
                    anchors.fill: parent
                    spacing: 16

                    Label {
                        text: "操作"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                    }

                    Item { Layout.fillWidth: true }

                    // 上下使能（单按钮切换）
                    Button {
                        text: isEnabled ? "下使能" : "上使能"
                        enabled: isConnected && !isConnecting
                        Material.background: isEnabled ? Material.Orange : Material.Green
                        Material.foreground: "white"
                        onClicked: {
                            if (isEnabled) {
                                disableRobot()
                            } else {
                                enableRobot()
                            }
                        }
                    }

                    // 清错
                    Button {
                        text: "清错"
                        enabled: isConnected && hasError && !isConnecting
                        Material.background: Material.Red
                        Material.foreground: "white"
                        onClicked: clearError()
                    }

                    // 重置传感器
                    Button {
                        text: "重置传感器"
                        enabled: isConnected && !isConnecting
                        Material.background: Material.Blue
                        Material.foreground: "white"
                        onClicked: resetSensor()
                    }
                }
            }
        }
    }
    
    // 更新连接状态
    function updateConnectionStatus(connected, connecting) {
        isConnected = connected
        isConnecting = connecting
    }
    
    // 更新使能状态
    function updateEnableStatus(enabled) {
        isEnabled = enabled
    }
    
    // 更新错误状态
    function updateErrorStatus(hasError) {
        this.hasError = hasError
    }
    
    // 更新配置
    function updateConfig(ip, port) {
        currentIp = ip
        currentPort = port
    }
}