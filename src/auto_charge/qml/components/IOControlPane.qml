import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// IO控制组件
Item {
    id: ioControlPane
    property int ioIndex: 0
    property bool ioState: false
    property bool enabled: true
    property string title: "IO控制"
    signal ioChanged(int index, bool state)
    Layout.preferredHeight: 48
    
    RowLayout {
        anchors.fill: parent
        spacing: 12
        Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
        
        Label {
            text: title
            font.pixelSize: 14
            Layout.preferredWidth: 60
            Layout.alignment: Qt.AlignVCenter
        }
        ComboBox {
            id: ioIndexCombo
            model: ListModel {
                // 直接静态生成数字项，避免Component.onCompleted时机导致的显示异常
                ListElement { display: "0" }
                ListElement { display: "1" }
                ListElement { display: "2" }
                ListElement { display: "3" }
                ListElement { display: "4" }
                ListElement { display: "5" }
                ListElement { display: "6" }
                ListElement { display: "7" }
                ListElement { display: "8" }
                ListElement { display: "9" }
                ListElement { display: "10" }
                ListElement { display: "11" }
                ListElement { display: "12" }
                ListElement { display: "13" }
                ListElement { display: "14" }
                ListElement { display: "15" }
                ListElement { display: "16" }
                ListElement { display: "17" }
                ListElement { display: "18" }
                ListElement { display: "19" }
                ListElement { display: "20" }
            }
            textRole: "display"
            currentIndex: ioIndex
            enabled: ioControlPane.enabled
            Layout.preferredWidth: 100
            Layout.alignment: Qt.AlignVCenter
            onCurrentIndexChanged: {
                if (ioControlPane.ioIndex !== currentIndex) {
                    ioControlPane.ioIndex = currentIndex
                }
            }
        }
        
        // 状态显示
        Label {
            text: ioState ? "ON" : "OFF"
            font.pixelSize: 14
            font.weight: Font.Medium
            color: ioState ? Material.Green : Material.Red
            Layout.preferredWidth: 35
            Layout.alignment: Qt.AlignVCenter
        }
        
        Switch {
            id: ioSwitch
            checked: ioState
            enabled: ioControlPane.enabled
            Layout.alignment: Qt.AlignVCenter
            
            onCheckedChanged: {
                ioControlPane.ioState = checked
                ioChanged(ioIndex, checked)
            }
        }
        
        // 填充剩余空间，确保左对齐
        Item { Layout.fillWidth: true }
    }
    onIoIndexChanged: {
        if (ioIndexCombo.currentIndex !== ioIndex) {
            ioIndexCombo.currentIndex = ioIndex
        }
    }
    onIoStateChanged: {
        if (ioSwitch.checked !== ioState) {
            ioSwitch.checked = ioState
        }
    }
} 