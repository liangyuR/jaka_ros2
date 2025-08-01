import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// 状态行组件
Pane {
    id: statusBar
    Layout.fillWidth: true
    Layout.preferredHeight: 48
    Material.elevation: 2
    Material.background: Material.surface
    
    // 状态更新时间戳
    property string currentTime: Qt.formatDateTime(new Date(), "hh:mm:ss")
    
    // 定时器，每秒更新时间戳
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            currentTime = Qt.formatDateTime(new Date(), "hh:mm:ss")
        }
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: 16
        
        // 机械臂状态图标
        Rectangle {
            width: 12
            height: 12
            radius: 6
            color: robotManager.connected ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Red, Material.Shade500)
            
            BusyIndicator {
                anchors.centerIn: parent
                width: 8
                height: 8
                running: robotManager.connecting
                visible: robotManager.connecting
            }
        }
        
        // 机械臂状态文本
        Label {
            text: {
                if (robotManager.connecting) return "机械臂: 连接中..."
                else if (robotManager.connected) return "机械臂: 已连接"
                else return "机械臂: 未连接"
            }
            font.pixelSize: 12
            font.weight: Font.Medium
        }
        
                    // 分隔符
            Rectangle {
                width: 1
                height: 16
                color: Material.color(Material.Grey, Material.Shade300)
            }
        
        // 状态信息组
        RowLayout {
            spacing: 12
            visible: robotManager.connected
            
            // 电源状态
            RowLayout {
                spacing: 4
                
                Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: robotManager.poweredOn ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: "电源"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.poweredOn ? "已上电" : "未上电"
                    font.pixelSize: 11
                    color: robotManager.poweredOn ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Grey, Material.Shade500)
                }
            }
            
            // 使能状态
            RowLayout {
                spacing: 4
                
                Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: robotManager.enabled ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: "使能"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.enabled ? "已使能" : "未使能"
                    font.pixelSize: 11
                    color: robotManager.enabled ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Grey, Material.Shade500)
                }
            }
            
            // 错误状态
            RowLayout {
                spacing: 4
                
                Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: robotManager.error ? Material.color(Material.Red, Material.Shade500) : Material.color(Material.Green, Material.Shade500)
                }
                
                Label {
                    text: "错误"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.error ? "有错误" : "正常"
                    font.pixelSize: 11
                    color: robotManager.error ? Material.color(Material.Red, Material.Shade500) : Material.color(Material.Green, Material.Shade500)
                }
            }
            
            // 急停状态
            RowLayout {
                spacing: 4
                
                Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: robotManager.emergencyStop ? Material.color(Material.Red, Material.Shade500) : Material.color(Material.Green, Material.Shade500)
                }
                
                Label {
                    text: "急停"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.emergencyStop ? "已急停" : "正常"
                    font.pixelSize: 11
                    color: robotManager.emergencyStop ? Material.color(Material.Red, Material.Shade500) : Material.color(Material.Green, Material.Shade500)
                }
            }
            
            // 碰撞状态
            RowLayout {
                spacing: 4
                
                Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: robotManager.protectiveStop ? Material.color(Material.Red, Material.Shade500) : Material.color(Material.Green, Material.Shade500)
                }
                
                Label {
                    text: "碰撞"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.protectiveStop ? "检测到" : "正常"
                    font.pixelSize: 11
                    color: robotManager.protectiveStop ? Material.color(Material.Red, Material.Shade500) : Material.color(Material.Green, Material.Shade500)
                }
            }
            
            // 到位状态
            RowLayout {
                spacing: 4
                
                Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: robotManager.inpos ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Orange, Material.Shade500)
                }
                
                Label {
                    text: "到位"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.inpos ? "到位" : "未到位"
                    font.pixelSize: 11
                    color: robotManager.inpos ? Material.color(Material.Green, Material.Shade500) : Material.color(Material.Orange, Material.Shade500)
                }
            }
        }
        
        Item { Layout.fillWidth: true }
        
        // 右侧信息组
        RowLayout {
            spacing: 12
            
            // 运动倍率
            RowLayout {
                spacing: 4
                visible: robotManager.connected
                
                Label {
                    text: "倍率:"
                    font.pixelSize: 11
                    color: Material.color(Material.Grey, Material.Shade500)
                }
                
                Label {
                    text: robotManager.rapidrate + "%"
                    font.pixelSize: 11
                }
            }
            
            // 分隔符
            Rectangle {
                width: 1
                height: 16
                color: Material.color(Material.Grey, Material.Shade300)
                visible: robotManager.connected
            }
            
            // 时间戳
            Label {
                text: "更新时间: " + currentTime
                font.pixelSize: 11
                color: Material.color(Material.Grey, Material.Shade500)
            }
        }
    }
} 