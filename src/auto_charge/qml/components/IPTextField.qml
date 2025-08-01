import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material

TextField {
    id: ipTextField
    
    property string ipAddress: "127.0.0.1"
    
    text: ipAddress
    placeholderText: "请输入IP地址"    
    
    // 宽松的IP地址验证器，允许输入过程中的中间状态
    validator: RegularExpressionValidator {
        regularExpression: /^(\d{0,3}\.){0,3}\d{0,3}$/
    }
    
    // 输入处理
    onTextChanged: {
        if (text.length > 0) {
            // 只允许数字和点
            let cleaned = text.replace(/[^\d.]/g, '');
            let parts = cleaned.split('.');
            
            // 处理每个部分
            for (let i = 0; i < parts.length; i++) {
                let part = parts[i];
                
                // 如果部分为空，跳过处理
                if (part === "") continue;
                
                // 限制每段最大长度为3位
                if (part.length > 3) {
                    parts[i] = part.substring(0, 3);
                }
                
                // 检查数值范围 [0, 255]
                let num = parseInt(parts[i]);
                if (!isNaN(num)) {
                    if (num > 255) {
                        parts[i] = "255";
                    } else if (num < 0) {
                        parts[i] = "0";
                    }
                }
            }
            
            let result = parts.join('.');
            if (result !== text) {
                ipTextField.text = result;
                ipTextField.cursorPosition = result.length;
            }
            
            // 更新ipAddress属性
            ipAddress = result;
        }
    }
    
    // 键盘处理
    Keys.onPressed: function(event) {
        if (event.key === Qt.Key_Period || event.key === Qt.Key_Comma) {
            event.accepted = true;
            let pos = cursorPosition;
            let newText = text.substring(0, pos) + "." + text.substring(pos);
            text = newText;
            cursorPosition = pos + 1;
        }
    }
    
    // 当ipAddress属性改变时更新text
    onIpAddressChanged: {
        if (text !== ipAddress) {
            text = ipAddress;
        }
    }
}
