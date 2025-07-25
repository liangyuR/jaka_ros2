import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import "./components"

Item {
    id: alson_client_ui
    
    // 信号定义
    signal restartCameraNode()
    signal saveConfig(string ip, int port)
    signal saveMapping(string chargeStation, string connector, string placement, string chargeBox)
    
    // 属性
    property string currentIp: "127.0.0.1"
    property int currentPort: 59999
    property bool isConnected: false
    property bool isConnecting: false
    
    // 项目映射属性
    property string chargeStationId: "Charge01"
    property string connectorId: "Connector01"
    property string placementId: "Place01"
    property string chargeBoxId: "Box01"
    
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
                    
                    // 连接时间
                    Label {
                        text: "最后连接: " + Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
                        font.pixelSize: 12
                        color: Material.Grey
                        visible: isConnected
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

                    ListView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        spacing: 16
                        model: ListModel {
                            ListElement { key: "充电座"; value: "Charge01"; placeholder: "输入充电座标识" }
                            ListElement { key: "连接件"; value: "Connector01"; placeholder: "输入连接件标识" }
                            ListElement { key: "放置位"; value: "Place01"; placeholder: "输入放置位标识" }
                            ListElement { key: "充电箱"; value: "Box01"; placeholder: "输入充电箱标识" }
                        }
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
                                    text: key
                                    Layout.preferredWidth: 100
                                    font.pixelSize: 16
                                    font.weight: Font.Medium
                                    color: Material.foreground
                                    verticalAlignment: Text.AlignVCenter
                                }
                                TextField {
                                    text: value
                                    placeholderText: placeholder
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
                    
                    Item { Layout.fillWidth: true }
                    
                    // 保存配置按钮
                    Button {
                        text: "保存配置并重启相机上位机"
                        Material.background: Material.primary
                        Material.foreground: "white"
                        onClicked: {
                            saveConfig(netConfig.ip, netConfig.port)
                            var mappingData = mappingConfig.getMappingData()
                            saveMapping(mappingData[0].value, mappingData[1].value, 
                                      mappingData[2].value, mappingData[3].value)
                            restartCameraNode()
                        }
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
    
    // 更新配置
    function updateConfig(ip, port) {
        currentIp = ip
        currentPort = port
        netConfig.ip = ip
        netConfig.port = port
    }
    
    // 更新项目映射信息
    function updateMapping(chargeStation, connector, placement, chargeBox) {
        chargeStationId = chargeStation
        connectorId = connector
        placementId = placement
        chargeBoxId = chargeBox
        
        var newMappingData = [
            {"key": "充电座", "value": chargeStation, "placeholder": "输入充电座标识"},
            {"key": "连接件", "value": connector, "placeholder": "输入连接件标识"},
            {"key": "放置位", "value": placement, "placeholder": "输入放置位标识"},
            {"key": "充电箱", "value": chargeBox, "placeholder": "输入充电箱标识"}
        ]
        mappingConfig.setMappingData(newMappingData)
    }
}
