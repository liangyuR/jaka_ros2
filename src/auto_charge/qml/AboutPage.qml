import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// 关于页面组件
Item {
    ScrollView {
        anchors.fill: parent
        clip: true
        
        ColumnLayout {
            width: parent.width
            spacing: 24
            
            Label {
                text: "关于"
                font.pixelSize: 28
                font.weight: Font.Medium
                Layout.topMargin: 24
                Layout.leftMargin: 24
            }
            
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 16
                    
                    Label {
                        text: "应用信息"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                    }
                    
                    Label {
                        text: "Auto Charge Manager"
                        font.pixelSize: 16
                        font.weight: Font.Medium
                    }
                    
                    Label {
                        text: "版本: 1.0.0"
                    }
                    
                    Label {
                        text: "基于 Qt Quick Controls 2 - Material"
                    }
                    
                    Label {
                        text: "© 2025 JX Team"
                    }
                    
                    Button {
                        text: "检查更新"
                        Layout.alignment: Qt.AlignRight
                        Material.background: Material.accent
                        Material.foreground: Material.onAccent
                        
                        onClicked: {
                            // 检查更新逻辑
                            console.log("检查更新...")
                        }
                    }
                }
            }
            
            // 添加更多信息卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 16
                    
                    Label {
                        text: "技术信息"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                    }
                    
                    Label {
                        text: "Qt 版本: " + Qt.qmlVersion
                    }
                    
                    Label {
                        text: "构建时间: " + Qt.application.arguments[0]
                    }
                    
                    Label {
                        text: "平台: " + Qt.platform.os
                    }
                }
            }
            
            // 联系方式卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 16
                    
                    Label {
                        text: "联系我们"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                    }
                    
                    Label {
                        text: "邮箱: support@autocharge.com"
                    }
                    
                    Label {
                        text: "官网: https://www.autocharge.com"
                    }
                    
                    Label {
                        text: "技术支持: 400-123-4567"
                    }
                }
            }
        }
    }
}

