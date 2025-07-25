import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Dialog {
    id: floatPoseDialog
    modal: true
    title: "输入 fl pose"
    standardButtons: Dialog.Ok | Dialog.Cancel
    
    // 属性
    property string flPoseInput: ""
    property var flPoseList: []
    property string placeholderText: "如: 1.0, 2.5, 3.14"
    property string descriptionText: "请输入 fl pose (以逗号分隔的浮点数列表):"
    
    // 信号
    signal poseAccepted(var poseList)
    signal poseRejected()
    
    ColumnLayout {
        spacing: 12
        width: 300
        
        Label {
            text: descriptionText
            wrapMode: Text.WordWrap
        }
        
        TextField {
            id: flPoseField
            placeholderText: placeholderText
            text: floatPoseDialog.flPoseInput
            Layout.fillWidth: true
            onTextChanged: floatPoseDialog.flPoseInput = text
        }
    }
    
    onAccepted: {
        // 解析输入为 float 数组
        var arr = flPoseField.text.split(",").map(function(item) {
            return parseFloat(item.trim())
        }).filter(function(val) {
            return !isNaN(val)
        })
        flPoseList = arr
        poseAccepted(flPoseList)
    }
    
    onRejected: {
        // 取消时清空输入
        flPoseInput = ""
        flPoseField.text = ""
        poseRejected()
    }
    
    // 公共方法
    function openDialog() {
        open()
    }
    
    function closeDialog() {
        close()
    }
    
    function clearInput() {
        flPoseInput = ""
        flPoseField.text = ""
        flPoseList = []
    }
} 