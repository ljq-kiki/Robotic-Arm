/**
 * 静态素材统一入口（文件放在 public/ 下，这里只写 URL 路径）。
 * 以后要换图：替换 public 里的文件，或改这里的路径即可。
 */
export const mediaAssets = {
  /** 右侧「3D ROBOT MODEL」卡片：真实机械臂预览照 */
  robot3dPreview: '/media/robot-3d-preview.png',

  /** Install 步骤：工具安装引导图（左→右 1、2、3） */
  installToolGuide: [
    { src: '/install-tool-guide/1.png', alt: 'Tool installation step 1' },
    { src: '/install-tool-guide/2.png', alt: 'Tool installation step 2' },
    { src: '/install-tool-guide/3.png', alt: 'Tool installation step 3' },
  ],

  /** Test 页碰撞提示弹窗里的示意图 */
  toolTestHintIllustration: '/placeholders/chopsticks-illustration.png',

  /**
   * 庆祝动图路径（浏览器对 GIF 会自动循环）。
   * 请把「真正的」GIF 放在 public/media/all-games-celebration.gif。
   * 若尚未提供 GIF，会 404 后自动回退到 allGamesCelebrationStill。
   */
  allGamesCelebrationGif: '/media/all-games-celebration.gif',

  /** 静帧备用（JPEG，扩展名与内容一致，避免伪 .gif 无法动画） */
  allGamesCelebrationStill: '/media/all-games-celebration.jpg',

  /** @deprecated 使用 <CelebrationImage />，保留路径供检索 */
  assemblyCelebrationGif: '/media/all-games-celebration.gif',

  /** Execute 页右侧卡片：Robot Workspace 示意图 */
  robotWorkspaceMap: '/placeholders/robot-workspace-map.png',

  /** Execute 页 E-stop 图标（指令项与装饰图共用） */
  emergencyStopIcon: '/icons/emergency-stop.png',

  /** Execute 页第一条提示的积木图标 */
  blockBrickIcon: '/icons/block-brick.png',

  /** 弹窗提示：灯泡图标 */
  hintBulbIcon: '/icons/hint-bulb.png',

  /** 弹窗问题：思考图标 */
  hintThinkingIcon: '/icons/hint-thinking.png',

  /** Execute 页安全区域示意图 */
  executionWarningMapClean: '/placeholders/execution-warning-map-clean.png',

  /** Assembly 少路径点提示弹窗里的道路绕障示意图 */
  waypointDrivingAroundRock: '/placeholders/waypoint-driving-around-rock.png',

  /** Assembly E-stop 提示弹窗里的按钮示意图 */
  assemblyEstopButton: '/placeholders/assembly-estop-button.png',

  /** Assembly 坐标系错误提示弹窗里的参考系示意图 */
  assemblyDirectionReferenceFrame: '/placeholders/assembly-direction-reference-frame.png',

  /** Assembly 奇异点提示弹窗里的警告示意图 */
  assemblySingularityWarning: '/placeholders/assembly-singularity-warning.png',
}
