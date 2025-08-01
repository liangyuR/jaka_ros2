import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

import "./robot_pages"
import "./components"

Item {
    id: robot_control_ui

    Component.onCompleted: {
        // 默认选中基础页面
        navListView.currentIndex = 0
        stackView.replace(baseSettingPage)
    }

    // 主布局
    RowLayout {
        anchors.fill: parent
        spacing: 0
        
        // 左侧导航栏
        Pane {
            Layout.preferredWidth: 280
            Layout.fillHeight: true
            Material.elevation: 1
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 16
                
                // 标题
                Label {
                    text: "机械臂控制"
                    font.pixelSize: 24
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignHCenter
                    Layout.topMargin: 16
                }
                
                // 导航列表
                ListView {
                    id: navListView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    model: ListModel {
                        ListElement { name: "基础设置"; page: "baseSettingPage" }
                    }
                    
                    delegate: ItemDelegate {
                        width: parent.width
                        height: 56
                        
                        RowLayout {
                            anchors.fill: parent
                            anchors.leftMargin: 16
                            anchors.rightMargin: 16
                            spacing: 16
                            
                            Label {
                                text: model.icon
                                font.pixelSize: 20
                            }
                            
                            Label {
                                text: model.name
                                font.pixelSize: 16
                                Layout.fillWidth: true
                            }
                            
                            Rectangle {
                                width: 4
                                height: parent.height - 16
                                radius: 2
                                color: Material.accent
                                visible: navListView.currentIndex === index
                            }
                        }
                        
                        background: Rectangle {
                            color: navListView.currentIndex === index ? 
                                   Material.accentColor : "transparent"
                            opacity: navListView.currentIndex === index ? 0.1 : 0
                        }
                        
                        onClicked: {
                            navListView.currentIndex = index
                            switch(model.page) {
                                case "baseSettingPage":
                                    stackView.replace(baseSettingPage)
                                    break
                            }
                        }
                    }
                }
            }
        }
        
        // 右侧内容区域
        StackView {
            id: stackView
            Layout.fillWidth: true
            Layout.fillHeight: true
            initialItem: statusPage
            
            // 页面切换动画
            replaceEnter: Transition {
                PropertyAnimation {
                    property: "opacity"
                    from: 0
                    to: 1
                    duration: 200
                }
            }
            replaceExit: Transition {
                PropertyAnimation {
                    property: "opacity"
                    from: 1
                    to: 0
                    duration: 200
                }
            }
        }
    }
    
    // 页面组件定义
    Component {
        id: baseSettingPage
        BaseSettingPage {}
    }
}