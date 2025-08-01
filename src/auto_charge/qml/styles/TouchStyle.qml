import QtQuick
import QtQuick.Controls.Material

// 触控优化的样式组件
QtObject {
    // 触控友好的尺寸
    readonly property int touchButtonHeight: 56
    readonly property int touchButtonMinWidth: 120
    readonly property int touchSpacing: 16
    readonly property int touchMargin: 24
    
    // 触控友好的字体大小
    readonly property int touchFontSize: 18
    readonly property int touchTitleFontSize: 24
    
    // 触控友好的颜色
    readonly property color touchPrimaryColor: Material.primary
    readonly property color touchAccentColor: Material.accent
    readonly property color touchBackgroundColor: Material.background
    readonly property color touchSurfaceColor: Material.surface
    
    // 触控友好的阴影
    readonly property int touchElevation: 2
    
    // 触控友好的圆角
    readonly property int touchRadius: 8
    
    // 触控友好的动画时长
    readonly property int touchAnimationDuration: 200
} 