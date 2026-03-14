# hardware/

硬件通讯逻辑目录，由负责硬件的同学实现。

## 约定

- 实现 `src/services/Bridge.ts` 中定义的 **`IHardwareBridge`** 接口。
- 接口方法包括：
  - `connectDevice()`：连接设备
  - `disconnectDevice()`：断开连接
  - `onDataReceived(callback)`：注册“收到硬件数据”的回调
  - `getStatus()`：返回当前设备/连接状态
  - （可选）`sendCommand(command, payload)`：下发指令

UI 通过 `services/Bridge` 的接口调用硬件能力，本目录提供真实实现并注入到应用（例如在 `main.tsx` 或上层 Provider 中替换 `createMockBridge()`）。
