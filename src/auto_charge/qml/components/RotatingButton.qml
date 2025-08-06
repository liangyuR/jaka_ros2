import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material

Button {
    id: rotatingButton
    
    property bool isRotating: false
    
    // 加载指示器
    Rectangle {
        id: loadingIndicator
        width: 16
        height: 16
        radius: 8
        color: "transparent"
        border.color: Material.foreground
        border.width: 2
        anchors.centerIn: parent
        visible: isRotating
        
        RotationAnimation on rotation {
            from: 0
            to: 360
            duration: 1000
            loops: Animation.Infinite
            running: isRotating
        }
    }
    
    // 当显示加载指示器时隐藏文字
    contentItem: Text {
        text: rotatingButton.text
        font: rotatingButton.font
        color: rotatingButton.Material.foreground
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        visible: !isRotating
    }
} 