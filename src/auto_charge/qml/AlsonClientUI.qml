import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Dialogs
import "./components"

Item {
    id: alson_client_ui

    readonly property color materialGreen: Material.color(Material.Green, Material.Shade500)
    readonly property color materialRed: Material.color(Material.Red, Material.Shade500)
    readonly property color materialOrange: Material.color(Material.Orange, Material.Shade500)

    signal restartCameraNode
    signal saveConfig(string ip, int port)
    signal saveMapping(string chargeStation, string connector, string placement, string chargeBox)

    Component.onCompleted: {
        console.log("AlsonClientUI component completed, initializing...");
        console.log("cameraController available:", cameraController ? "yes" : "no");
    }

    ScrollView {
        anchors.fill: parent
        clip: true

        ColumnLayout {
            width: parent.width
            spacing: 24

            // 页面标题
            Label {
                text: "ALSON 相机配置"
                font.pixelSize: 28
                font.weight: Font.Medium
                color: Material.primary
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
                        color: {
                            if (cameraController.isConnecting) {
                                return materialOrange;  // 橙色 - 连接中
                            } else if (cameraController.isConnected) {
                                return materialGreen;   // 绿色 - 已连接
                            } else {
                                return materialRed;     // 红色 - 未连接
                            }
                        }
                        BusyIndicator {
                            anchors.centerIn: parent
                            width: 12
                            height: 12
                            running: cameraController.isConnecting
                            visible: cameraController.isConnecting
                        }
                    }

                    // 状态文本
                    Label {
                        text: {
                            if (cameraController.isConnecting)
                                return "连接中...";
                            else if (cameraController.isConnected)
                                return "已连接";
                            else
                                return "未连接";
                        }
                        font.pixelSize: 16
                        font.weight: Font.Medium
                    }

                    Item {
                        Layout.fillWidth: true
                    }

                    // 连接控制按钮
                    RowLayout {
                        spacing: 8

                        // 连接时间
                        Label {
                            text: "最后连接: " + Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
                            font.pixelSize: 12
                            color: Material.Grey
                            visible: cameraController.isConnected
                        }

                        Button {
                            text: "连接"
                            enabled: !cameraController.isConnected && !cameraController.isConnecting
                            Material.background: Material.Green
                            Material.foreground: "white"

                            onClicked: {
                                cameraController.ConnectCamera();
                            }
                        }

                        Button {
                            text: "断开"
                            enabled: cameraController.isConnected && !cameraController.isConnecting
                            Material.background: Material.Red
                            Material.foreground: "white"

                            onClicked: {
                                cameraController.DisconnectCamera();
                            }
                        }

                        Button {
                            text: "重启"
                            enabled: !cameraController.isConnecting
                            Material.background: Material.Orange
                            Material.foreground: "white"

                            onClicked: {
                                cameraController.RestartCamera();
                            }
                        }
                    }
                }
            }

            // 网络配置卡片
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 1
                Layout.preferredWidth: 520

                NetworkConfigPane {
                    Layout.fillWidth: true
                    id: netConfig
                    ip: cameraController.currentHost
                    port: cameraController.currentPort
                    connectionTimeout: cameraController.waitForConnectionTimeout
                    onConfigChanged: function (ip, port, timeout) {
                            cameraController.currentHost = ip;
                            cameraController.currentPort = port;
                            cameraController.waitForConnectionTimeout = timeout;
                    }
                }
            }

            // 项目映射信息
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 2
                Layout.preferredHeight: 550
                Layout.preferredWidth: 520

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 8

                    Label {
                        text: "项目映射信息"
                        font.pixelSize: 20
                        font.weight: Font.DemiBold
                        color: Material.foreground
                        Layout.topMargin: 20
                        Layout.leftMargin: 0
                        Layout.bottomMargin: 12
                    }

                    // 运行超时时间
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 16

                        Label {
                            text: "运行超时时间"
                        }

                        SpinBox {
                            id: runProjectTimeoutSpinBox
                            Layout.fillWidth: true
                            from: 1
                            to: 60
                            value: cameraController.runProjectTimeout
                            onValueChanged: {
                                cameraController.runProjectTimeout = value;
                            }
                        }
                    }

                    ListView {
                        id: mappingListView
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        spacing: 16
                        model: [
                            {
                                key: "charge_station",
                                label: "充电座",
                                propertyName: "chargeStationId",
                                placeholder: "输入充电座标识"
                            },
                            {
                                key: "connector",
                                label: "连接件",
                                propertyName: "connectorId",
                                placeholder: "输入连接件标识"
                            },
                            {
                                key: "placement",
                                label: "放置位",
                                propertyName: "placementId",
                                placeholder: "输入放置位标识"
                            },
                            {
                                key: "charge_box",
                                label: "充电箱",
                                propertyName: "chargeBoxId",
                                placeholder: "输入充电箱标识"
                            }
                        ]
                        delegate: Rectangle {
                            width: ListView.view.width - 32
                            height: 80
                            radius: 10
                            border.color: Material.dividerColor
                            border.width: 1
                            anchors.horizontalCenter: parent ? parent.horizontalCenter : undefined
                            Material.elevation: 1
                            RowLayout {
                                anchors.fill: parent
                                anchors.margins: 16
                                spacing: 20
                                Label {
                                    text: modelData.label
                                    Layout.preferredWidth: 100
                                    font.pixelSize: 16
                                    font.weight: Font.Medium
                                    color: Material.foreground
                                    verticalAlignment: Text.AlignVCenter
                                }
                                TextField {
                                    text: cameraController[modelData.propertyName]
                                    onTextChanged: {
                                        cameraController[modelData.propertyName] = text;
                                    }
                                    placeholderText: modelData.placeholder
                                    font.pixelSize: 15
                                    verticalAlignment: Text.AlignVCenter
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 200
                                    background: Rectangle {
                                        radius: 6
                                        border.color: Material.dividerColor
                                        border.width: 1
                                    }
                                }
                                Button {
                                    text: "测试"
                                    font.pixelSize: 15
                                    Layout.preferredWidth: 100
                                    Layout.alignment: Qt.AlignVCenter
                                    Material.background: Material.accent
                                    Material.foreground: "white"

                                    onClicked: {
                                        cameraController.RunProject(
                                            cameraController[modelData.propertyName],
                                            [0, 0, 0, 0, 0, 0]); // TODO(@liangyu) for test
                                    }
                                }
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

                    Item {
                        Layout.fillWidth: true
                    }

                    Button {
                        text: "加载配置(yaml)"
                        Material.background: Material.primary
                        Material.foreground: "white"
                        onClicked: yamlFileDialog.open()
                    }

                    // 保存配置按钮
                    Button {
                        text: "保存配置"
                        Material.background: Material.primary
                        Material.foreground: "white"
                        onClicked: {
                            cameraController.saveConfig();
                        }
                    }
                }
            }
        }
    }

    // 文件选择对话框
    FileDialog {
        id: yamlFileDialog
        title: "选择YAML配置文件"
        nameFilters: ["YAML文件 (*.yaml *.yml)", "所有文件 (*.*)"]
        onAccepted: {
            console.log("文件选择对话框 accepted");
            console.log("selectedFile:", selectedFile);
            console.log("cameraController:", cameraController);
            if (selectedFile && cameraController) {
                console.log("选择的文件:", selectedFile);
                cameraController.loadConfig(selectedFile);
            } else {
                console.log("未选择文件或cameraController未初始化");
            }
        }
    }
}
