import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

import "./components"

ScrollView {
    id: scrollView
    clip: true
    
    ColumnLayout {
        width: parent.width
        spacing: 24
        
        // 页面标题
        Label {
            text: "Demo"
            font.pixelSize: 28
            font.weight: Font.Medium
            Layout.topMargin: 24
            Layout.leftMargin: 24
        }

        // 关节位置输入 & 移动
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2
            Layout.preferredWidth: scrollView.width - 48

            ColumnLayout {
                Layout.fillHeight: true
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                RowLayout {
                    TextField {
                        id: jointInput
                        placeholderText: "输入关节角度, JSON数组格式 (如: [0,45,90,0,0,0])"
                        Layout.preferredWidth: 720
                    }
                    Button {
                        text: "移动到关节位置"
                        Material.background: Material.accent
                        Material.foreground: "white"
                        onClicked: {
                            var joints
                            try {
                                joints = JSON.parse(jointInput.text)
                            } catch (e) {
                                jointInput.text = ""
                                jointInput.placeholderText = "格式错误, 请输入JSON数组"
                                return
                            }
                            if (!Array.isArray(joints) || joints.length !== 6 || joints.some(function(val){ return typeof val !== "number" || isNaN(val) })) {
                                jointInput.text = ""
                                jointInput.placeholderText = "格式错误, 请输入6个数字的JSON数组"
                                return
                            }
                            if (typeof robotManager !== "undefined" && robotManager.moveJoints) {
                                robotManager.moveJoints(joints)
                            } else {
                                console.log("robotManager.moveJoints(joints) 未实现")
                            }
                        }
                    }
                }
            }
        }

        // IO控制
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2
            
            RowLayout {
                anchors.fill: parent
                spacing: 24
                Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter

                // 通用IO控制
                IOControlPane {
                    title: "按钮"
                    id: generalIOControl
                    Layout.preferredWidth: 300
                    Layout.preferredHeight: 48
                    Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
                    ioIndex: 1
                    ioState: false
                    
                    onIoChanged: function(index, state) {
                        console.log("通用IO状态改变:", index, state)
                        // 这里可以调用robotManager的IO控制方法
                        // robotManager.setIO(index, state)
                    }
                }
                
                // IO2 连接件控制
                IOControlPane {
                    title: "连接件"
                    id: connectorIOControl
                    Layout.preferredWidth: 300
                    Layout.preferredHeight: 48
                    Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
                    ioIndex: 2
                    ioState: false
                    
                    onIoChanged: function(index, state) {
                        console.log("连接件IO状态改变:", index, state)
                        // 这里可以调用robotManager的IO控制方法
                        // robotManager.setIO(index, state)
                    }
                }
                
                // 填充剩余空间
                Item { Layout.fillWidth: true }
            }
        }

        // 场景控制
        Pane{
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2

            ColumnLayout{
                Layout.fillHeight: true
                Layout.preferredWidth: scrollView.width - 48
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                Label {
                    text: "场景控制"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignLeft
                }

                // 横向排列场景控制按钮
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 24
                    Material.foreground: "white"

                    Button {
                        text: "加载默认场景"
                        Material.background: Material.primary
                        font.pixelSize: 18
                        Layout.preferredWidth: 200
                        Layout.alignment: Qt.AlignHCenter
                        onClicked: {
                            robotManager.InitScene()
                            console.log("加载默认场景")
                        }
                    }
                }
            }
        }
        
        // 取拍照位
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2

            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: scrollView.width - 48
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                Label {
                    text: "取拍照位"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignLeft
                }

                // 横向排列所有操作按钮
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 24
                    Material.foreground: "white"

                    Button {
                        text: "识别充电箱并获取4个拍照位姿"
                        Material.background: Material.primary
                        font.pixelSize: 18
                        Layout.preferredWidth: 320
                        Layout.alignment: Qt.AlignHCenter
                        onClicked: {
                            // TODO: 调用识别充电箱并获取4个拍照位姿的逻辑
                            // 例如: robotManager.detectChargerAndGetPhotoPoses()
                        }
                    }

                    Button {
                        text: "移动到拍照位1"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 180
                        onClicked: {
                            // TODO: 调用移动到充电座拍照位1的逻辑
                            // 例如: robotManager.moveToChargerPhotoPose(1)
                        }
                    }

                    Button {
                        text: "移动到拍照位2"
                        Material.foreground: "white"
                        Material.background: Material.Grey
                        font.pixelSize: 16
                        Layout.preferredWidth: 180
                        onClicked: {
                            // TODO: 调用移动到充电座拍照位2的逻辑
                            // 例如: robotManager.moveToChargerPhotoPose(2)
                        }
                    }

                    Button {
                        text: "连接拍照位1"
                        Material.foreground: "white"
                        Material.background: Material.Grey
                        font.pixelSize: 16
                        Layout.preferredWidth: 180
                        onClicked: {
                            // TODO: 调用充电座连接拍照位1的逻辑
                            // 例如: robotManager.moveToChargerConnectPhotoPose(1)
                        }
                    }

                    Button {
                        text: "连接拍照位2"
                        Material.foreground: "white"
                        Material.background: Material.Grey
                        font.pixelSize: 16
                        Layout.preferredWidth: 180
                        onClicked: {
                            // TODO: 调用充电座连接拍照位2的逻辑
                            // 例如: robotManager.moveToChargerConnectPhotoPose(2)
                        }
                    }
                }
            }
        }

        // 连接操作
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2

            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: scrollView.width - 48
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                // 标题
                Label {
                    text: "连接操作"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignLeft
                }

                // 横向排列连接操作按钮
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 24

                    Button {
                        text: "连接左充电位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用连接左充电位的逻辑
                            // 例如: robotManager.connectLeftChargingPosition()
                        }
                    }

                    Button {
                        text: "连接右充电位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用连接右充电位的逻辑
                            // 例如: robotManager.connectRightChargingPosition()
                        }
                    }

                    Button {
                        text: "连接左放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用连接左放置位的逻辑
                            // 例如: robotManager.connectLeftPlacementPosition()
                        }
                    }

                    Button {
                        text: "连接右放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用连接右放置位的逻辑
                            // 例如: robotManager.connectRightPlacementPosition()
                        }
                    }
                }
            }
        }

        // 释放操作
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2

            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: scrollView.width - 48
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                // 标题
                Label {
                    text: "释放操作"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignLeft
                }

                // 横向排列释放操作按钮
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 24
                    Material.foreground: "white"

                    Button {
                        text: "释放左充电位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用释放左充电位的逻辑
                            // 例如: robotManager.releaseLeftChargingPosition()
                        }
                    }

                    Button {
                        text: "释放右充电位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用释放右充电位的逻辑
                            // 例如: robotManager.releaseRightChargingPosition()
                        }
                    }

                    Button {
                        text: "释放左放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用释放左放置位的逻辑
                            // 例如: robotManager.releaseLeftPlacementPosition()
                        }
                    }

                    Button {
                        text: "释放右放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用释放右放置位的逻辑
                            // 例如: robotManager.releaseRightPlacementPosition()
                        }
                    }
                }
            }
        }

        // 插入操作
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2

            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: scrollView.width - 48
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                // 标题
                Label {
                    text: "插入操作"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignLeft
                }

                // 横向排列插入操作按钮
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 24
                    Material.foreground: "white"

                    Button {
                        text: "插入左放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用插入左放置位的逻辑
                            // 例如: robotManager.insertLeftPlacementPosition()
                        }
                    }

                    Button {
                        text: "插入右放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用插入右放置位的逻辑
                            // 例如: robotManager.insertRightPlacementPosition()
                        }
                    }

                    Button {
                        text: "插入左充电口"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用插入左充电口的逻辑
                            // 例如: robotManager.insertLeftChargingPort()
                        }
                    }

                    Button {
                        text: "插入右充电口"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用插入右充电口的逻辑
                            // 例如: robotManager.insertRightChargingPort()
                        }
                    }
                }
            }
        }

        // 拔出操作
        Pane {
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2

            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: scrollView.width - 48
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                // 标题
                Label {
                    text: "拔出操作"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                    Layout.alignment: Qt.AlignLeft
                }

                // 横向排列拔出操作按钮
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 24
                    Material.foreground: "white"

                    Button {
                        text: "拔出左放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用拔出左放置位的逻辑
                            // 例如: robotManager.extractLeftPlacementPosition()
                        }
                    }

                    Button {
                        text: "拔出右放置位"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用拔出右放置位的逻辑
                            // 例如: robotManager.extractRightPlacementPosition()
                        }
                    }

                    Button {
                        text: "拔出左充电口"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用拔出左充电口的逻辑
                            // 例如: robotManager.extractLeftChargingPort()
                        }
                    }

                    Button {
                        text: "拔出右充电口"
                        Material.background: Material.Grey
                        Material.foreground: "white"
                        font.pixelSize: 16
                        Layout.preferredWidth: 160
                        onClicked: {
                            // TODO: 调用拔出右充电口的逻辑
                            // 例如: robotManager.extractRightChargingPort()
                        }
                    }
                }
            }
        }

        // 双枪流程测试
        Pane {
            id: dualGunTestPane
            Layout.fillWidth: true
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Material.elevation: 2
            Layout.preferredWidth: scrollView.width - 48
            Layout.preferredHeight: 120
            RowLayout {
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                spacing: 32

                // 测试次数
                RowLayout {
                    spacing: 8
                    Label {
                        text: "测试次数:"
                        font.pixelSize: 15
                    }
                    SpinBox {
                        id: testCountSpinBox
                        from: 1
                        to: 100
                        value: 10
                        editable: true
                        Layout.preferredWidth: 100
                    }
                }

                // 控制按钮
                RowLayout {
                    spacing: 16
                    Button {
                        id: startBtn
                        text: "开始"
                        Material.background: Material.Green
                        Material.foreground: "white"
                        enabled: true
                        onClicked: {
                            // TODO: 实现开始逻辑
                        }
                    }
                    Button {
                        id: pauseBtn
                        text: "暂停"
                        Material.background: Material.Orange
                        Material.foreground: "white"
                        enabled: false
                        onClicked: {
                            // TODO: 实现暂停/继续逻辑
                        }
                    }
                    Button {
                        id: stopBtn
                        text: "结束"
                        Material.background: Material.Red
                        Material.foreground: "white"
                        enabled: false
                        onClicked: {
                            // TODO: 实现结束逻辑
                        }
                    }
                }

                // 当前进度
                RowLayout {
                    spacing: 8
                    Label {
                        text: "当前进度:"
                    }
                    ProgressBar {
                        value: 0
                        Layout.preferredWidth: 500
                    }
                    Label {
                        text: "0%"
                    }
                }
            }
        }
    }
} 