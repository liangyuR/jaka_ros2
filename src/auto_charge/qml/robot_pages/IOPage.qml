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
            text: "IO监控"
            font.pixelSize: 28
            font.weight: Font.Medium
            Layout.topMargin: 24
            Layout.leftMargin: 24
        }
        
        // 数字IO卡片
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 1

            ColumnLayout {
                anchors.fill: parent
                spacing: 16

                Label {
                    text: "数字IO状态"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                // 这里可以添加数字IO的显示
                Label {
                    text: "数字IO监控功能待实现"
                    color: Material.Grey
                    Layout.alignment: Qt.AlignHCenter
                }
            }
        }
        
        // 模拟IO卡片
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 1

            ColumnLayout {
                anchors.fill: parent
                spacing: 16

                Label {
                    text: "模拟IO状态"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                // 这里可以添加模拟IO的显示
                Label {
                    text: "模拟IO监控功能待实现"
                    color: Material.Grey
                    Layout.alignment: Qt.AlignHCenter
                }
            }
        }
        
        // 工具端IO卡片
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 1

            ColumnLayout {
                anchors.fill: parent
                spacing: 16

                Label {
                    text: "工具端IO状态"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                // 这里可以添加工具端IO的显示
                Label {
                    text: "工具端IO监控功能待实现"
                    color: Material.Grey
                    Layout.alignment: Qt.AlignHCenter
                }
            }
        }
    }
}
