import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: window
    visible: true
    width: 1280
    height: 1280
    title: "Auto Charge Settings"
    color: Material.background
    
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
                    text: "AutoCharge" + " " + "0.1.0" // TODO(@liangyu) fix version
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
                        ListElement { name: "常规"; icon: "⚙️"; page: "general" }
                        ListElement { name: "ALSON相机"; icon: "\uE412"; page: "alson" } // camera icon
                        ListElement { name: "Robot控制"; icon: "\uE87A"; page: "robot" } // robot icon
                        ListElement { name: "关于"; icon: "ℹ️"; page: "about" }
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
                                case "general":
                                    stackView.replace(generalPage)
                                    break
                                case "alson":
                                    stackView.replace(alsonPage)
                                    break
                                case "robot":
                                    stackView.replace(robotPage)
                                    break
                                case "about":
                                    stackView.replace(aboutPage)
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
            initialItem: generalPage
            
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
    
    // ALSON相机配置页面
    Component {
        id: alsonPage
        AlsonClientUI {
        }
    }

    // Robot控制页面
    Component {
        id: robotPage
        RobotControlUI {
        }
    }
    
    // 常规设置页面
    Component {
        id: generalPage
        ScrollView {
            clip: true
            
            ColumnLayout {
                width: parent.width
                spacing: 24
                
                // 页面标题
                Label {
                    text: "常规设置"
                    font.pixelSize: 28
                    font.weight: Font.Medium
                    Layout.topMargin: 24
                    Layout.leftMargin: 24
                }
                
                // 设置项
                Pane {
                    Layout.fillWidth: true
                    Layout.leftMargin: 24
                    Layout.rightMargin: 24
                    Material.elevation: 1
                    
                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 16
                        
                        Label {
                            text: "应用设置"
                            font.pixelSize: 18
                            font.weight: Font.Medium
                        }
                        
                        // 自动启动
                        RowLayout {
                            Layout.fillWidth: true
                            
                            Label {
                                text: "开机自动启动"
                                Layout.fillWidth: true
                            }
                            
                            Switch {
                                checked: true
                            }
                        }
                    }
                }
            }
        }
    }

    // 关于页面
    Component {
        id: aboutPage
        ScrollView {
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
                            text: "© 2024 Auto Charge Team"
                        }
                        
                        Button {
                            text: "检查更新"
                            Layout.alignment: Qt.AlignRight
                            Material.background: Material.accent
                            Material.foreground: Material.onAccent
                        }
                    }
                }
            }
        }
    }
}