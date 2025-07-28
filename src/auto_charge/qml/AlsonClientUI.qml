import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Dialogs
import "./components"

Item {
    id: alson_client_ui

    // Material 颜色定义
    readonly property color materialGreen: Material.color(Material.Green, Material.Shade500)
    readonly property color materialRed: Material.color(Material.Red, Material.Shade500)
    readonly property color materialOrange: Material.color(Material.Orange, Material.Shade500)

    // 信号定义
    signal restartCameraNode
    signal saveConfig(string ip, int port)
    signal saveMapping(string chargeStation, string connector, string placement, string chargeBox)

    // 直接绑定到CameraController的属性
    property var cameraController: autoChargeNode ? autoChargeNode.GetCameraController() : null

    // 页面创建时初始化
    Component.onCompleted: {
        console.log("AlsonClientUI component completed, initializing...");
    }

    // 主布局
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
                            if (cameraController && cameraController.isConnecting) {
                                return materialOrange;  // 橙色 - 连接中
                            } else if (cameraController && cameraController.isConnected) {
                                return materialGreen;   // 绿色 - 已连接
                            } else {
                                return materialRed;     // 红色 - 未连接
                            }
                        }
                        BusyIndicator {
                            anchors.centerIn: parent
                            width: 12
                            height: 12
                            running: cameraController ? cameraController.isConnecting : false
                            visible: cameraController ? cameraController.isConnecting : false
                        }
                    }

                    // 状态文本
                    Label {
                        text: {
                            if (cameraController && cameraController.isConnecting)
                                return "连接中...";
                            else if (cameraController && cameraController.isConnected)
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
                            visible: cameraController ? cameraController.isConnected : false
                        }

                        Button {
                            text: "连接"
                            enabled: cameraController && !cameraController.isConnected && !cameraController.isConnecting
                            Material.background: Material.Green
                            Material.foreground: "white"

                            onClicked: {
                                autoChargeNode.GetCameraController().ConnectCamera();
                            }
                        }

                        Button {
                            text: "断开"
                            enabled: cameraController && cameraController.isConnected && !cameraController.isConnecting
                            Material.background: Material.Red
                            Material.foreground: "white"

                            onClicked: {
                                autoChargeNode.GetCameraController().DisconnectCamera();
                            }
                        }

                        Button {
                            text: "重启"
                            enabled: cameraController && !cameraController.isConnecting
                            Material.background: Material.Orange
                            Material.foreground: "white"

                            onClicked: {
                                autoChargeNode.GetCameraController().RestartCamera();
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

                NetworkConfigPane {
                    id: netConfig
                    ip: cameraController ? cameraController.currentHost : "127.0.0.1"
                    port: cameraController ? cameraController.currentPort : 54600
                    connectionTimeout: cameraController ? cameraController.waitForConnectionTimeout : 30
                    onConfigChanged: function (ip, port, timeout) {
                        if (cameraController) {
                            cameraController.setCurrentHost(ip);
                            cameraController.setCurrentPort(port);
                            cameraController.setWaitForConnectionTimeout(timeout);
                        }
                    }
                }
            }

            // 项目映射信息
            Pane {
                Layout.fillWidth: true
                Layout.leftMargin: 24
                Layout.rightMargin: 24
                Material.elevation: 2
                Layout.preferredHeight: 480
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
                        Layout.leftMargin: 24
                        Layout.bottomMargin: 12
                    }

                    // 运行超时时间
                    RowLayout {
                        Layout.fillWidth: true

                        Label {
                            text: "运行超时时间"
                        }

                        SpinBox {
                            id: runProjectTimeoutSpinBox
                            from: 1
                            to: 60
                            value: cameraController ? cameraController.runProjectTimeout : 30
                            onValueChanged: {
                                if (cameraController) {
                                    cameraController.runProjectTimeout = value;
                                }
                            }
                            Layout.fillWidth: true
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
                                    text: cameraController ? cameraController[modelData.propertyName] : ""
                                    onTextChanged: {
                                        if (cameraController) {
                                            cameraController[modelData.propertyName] = text;
                                        }
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
                                        // 直接调用相机控制器的更新参数方法
                                        cameraController.UpdateCameraParam();
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
                        onClicked: {
                            loadConfig();
                        }
                    }

                    // 保存配置按钮
                    Button {
                        text: "保存配置"
                        Material.background: Material.primary
                        Material.foreground: "white"
                        onClicked: {
                            saveConfig(netConfig.ip, netConfig.port);
                            // 使用cameraController的属性
                            var chargeStation = cameraController ? cameraController.chargeStationId : "";
                            var connector = cameraController ? cameraController.connectorId : "";
                            var placement = cameraController ? cameraController.placementId : "";
                            var chargeBox = cameraController ? cameraController.chargeBoxId : "";
                            saveMapping(chargeStation, connector, placement, chargeBox);
                        }
                    }
                }
            }
        }
    }

    // 更新连接状态
    function updateConnectionStatus(connected, connecting) {
        isConnected = connected;
        isConnecting = connecting;
    }

    // 更新配置
    function updateConfig(ip, port) {
        currentIp = ip;
        currentPort = port;
        netConfig.ip = ip;
        netConfig.port = port;
    }

    // 更新项目映射信息
    function updateMapping(chargeStation, connector, placement, chargeBox) {
        chargeStationId = chargeStation;
        connectorId = connector;
        placementId = placement;
        chargeBoxId = chargeBox;

        var newMappingData = [
            {
                "key": "充电座",
                "value": chargeStation,
                "placeholder": "输入充电座标识"
            },
            {
                "key": "连接件",
                "value": connector,
                "placeholder": "输入连接件标识"
            },
            {
                "key": "放置位",
                "value": placement,
                "placeholder": "输入放置位标识"
            },
            {
                "key": "充电箱",
                "value": chargeBox,
                "placeholder": "输入充电箱标识"
            }
        ];
        mappingConfig.setMappingData(newMappingData);
    }

    // 监听相机事件
    Connections {
        target: autoChargeNode

        function onCameraEvent(eventType, eventData) {
            console.log("Camera event:", eventType, eventData);

            if (eventType === "connection_status") {
                isConnected = eventData.connected;
                isConnecting = false;
                console.log("Connection status updated:", isConnected, eventData.message);
            } else if (eventType === "reconnect_status") {
                if (eventData.status === "connecting") {
                    isConnecting = true;
                    isConnected = false;
                } else if (eventData.status === "connected") {
                    isConnected = true;
                    isConnecting = false;
                } else if (eventData.status === "disconnected") {
                    isConnected = false;
                    isConnecting = false;
                }
                console.log("Reconnect status:", eventData.status, eventData.message);
            } else if (eventType === "data_received") {
                console.log("Data received:", eventData.data);
            } else if (eventType === "heartbeat_status") {
                console.log("Heartbeat status:", eventData.success, eventData.message);
            } else if (eventType === "project_status") {
                console.log("Project status:", eventData.success, eventData.message);
            } else if (eventType === "update_param_result") {
                if (eventData.success) {
                    console.log("Camera parameter updated successfully:", eventData.message);
                    // 可以在这里显示成功消息
                } else {
                    console.error("Camera parameter update failed:", eventData.message);
                    // 可以在这里显示错误消息
                }
            } else if (eventType === "update_param_error") {
                console.error("Camera parameter update error:", eventData.error);
                // 可以在这里显示错误消息
            } else if (eventType === "status_updated") {
                console.log("Status updated:", eventData);
                // 更新连接状态
                isConnected = eventData.connected || false;
                isConnecting = false;
            } else if (eventType === "config_updated") {
                console.log("Config updated:", eventData);
                // 更新配置显示
                if (eventData.host) {
                    netConfig.ip = eventData.host;
                    currentIp = eventData.host;
                }
                if (eventData.port) {
                    netConfig.port = eventData.port;
                    currentPort = eventData.port;
                }
            }
        }

        function onCameraStatusChanged(status) {
            console.log("Camera status changed:", status);

            if (status === "connected") {
                isConnected = true;
                isConnecting = false;
            } else if (status === "disconnected") {
                isConnected = false;
                isConnecting = false;
            } else if (status === "connecting") {
                isConnecting = true;
                isConnected = false;
            }
        }
    }

    // 文件选择对话框
    FileDialog {
        id: yamlFileDialog
        title: "选择YAML配置文件"
        nameFilters: ["YAML文件 (*.yaml *.yml)", "所有文件 (*.*)"]
        onAccepted: {
            console.log("选择的文件:", selectedFile);
            loadYamlConfig(selectedFile);
        }
        onRejected: {
            console.log("文件选择被取消");
        }
    }

    // 加载配置函数
    function loadConfig() {
        yamlFileDialog.open();
    }

    // 加载YAML配置文件
    function loadYamlConfig(filePath) {
        console.log("正在加载YAML配置文件:", filePath);
        cameraController.loadConfig(filePath);
    }
}
