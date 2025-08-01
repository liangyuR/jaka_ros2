import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

ScrollView {
    clip: true
    
    ColumnLayout {
        width: parent.width
        spacing: 24
        
        // 页面标题
        Label {
            text: "状态监控"
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
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 16
                
                // 状态图标和文本
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 16
                    
                    // 状态图标
                    Rectangle {
                        width: 16
                        height: 16
                        radius: 8
                        color: robotManager.connected ? Material.Green : Material.Red
                        
                        BusyIndicator {
                            anchors.centerIn: parent
                            width: 12
                            height: 12
                            running: robotManager.connecting
                            visible: robotManager.connecting
                        }
                    }
                    
                    // 状态文本
                    Label {
                        text: {
                            if (robotManager.connecting) return "连接中..."
                            else if (robotManager.connected) return "已连接"
                            else return "未连接"
                        }
                        font.pixelSize: 16
                        font.weight: Font.Medium
                    }
                    
                    Item { Layout.fillWidth: true }
                    
                    // 使能状态
                    Label {
                        text: robotManager.enabled ? "已使能" : "未使能"
                        font.pixelSize: 12
                        color: robotManager.enabled ? Material.Green : Material.Grey
                        visible: robotManager.connected
                    }
                    
                    // 错误状态
                    Label {
                        text: "有错误"
                        font.pixelSize: 12
                        color: Material.Red
                        visible: robotManager.error
                    }
                }
                
                // 详细状态信息
                GridLayout {
                    Layout.fillWidth: true
                    columns: 2
                    rowSpacing: 8
                    columnSpacing: 16
                    
                    Label { text: "电源状态:"; font.pixelSize: 12; color: Material.Grey }
                    Label { text: robotManager.poweredOn ? "已上电" : "未上电"; font.pixelSize: 12 }
                    
                    Label { text: "运动倍率:"; font.pixelSize: 12; color: Material.Grey }
                    Label { text: robotManager.rapidrate; font.pixelSize: 12 }
                    
                    Label { text: "碰撞状态:"; font.pixelSize: 12; color: Material.Grey }
                    Label { 
                        text: robotManager.protectiveStop; 
                        font.pixelSize: 12
                        color: robotManager.protectiveStop ? Material.Red : Material.Green
                    }
                    
                    Label { text: "使能状态:"; font.pixelSize: 12; color: Material.Grey }
                    Label { 
                        text: robotManager.enabled ? "已使能" : "未使能"; 
                        font.pixelSize: 12
                        color: robotManager.enabled ? Material.Green : Material.Red
                    }

                    Label { text: "急停状态:"; font.pixelSize: 12; color: Material.Grey }
                    Label { 
                        text: robotManager.emergencyStop ? "已急停" : "未急停"; 
                        font.pixelSize: 12
                        color: robotManager.emergencyStop ? Material.Red : Material.Green
                    }

                    Label { text: "是否到位:"; font.pixelSize: 12; color: Material.Grey }
                    Label { 
                        text: robotManager.inpos ? "到位" : "未到位"; 
                        font.pixelSize: 12
                        color: robotManager.inpos ? Material.Green : Material.Red
                    }
                }
            }
        }
    }
} 