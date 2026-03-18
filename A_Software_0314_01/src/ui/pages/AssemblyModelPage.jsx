// import React, { useEffect, useRef, useState } from 'react'
// import {
//   HARDWARE_SIGNALS,
//   captureCurrentPoint,
//   initializeHardwareStore,
//   resetMockRobotToHome,
//   sendMockJogMove,
//   subscribeHardwareSignal,
//   startMockRun,
//   useHardwareStore,
// } from '../../services/useHardwareStore.ts'
// import { AssemblyModelPageView } from './AssemblyModelPageView.jsx'

// const EMPTY_POINT = { x: '', y: '', z: '', rx: '' }
// const ASSEMBLY_RUN_MS = 5000
// const COLLISION_SIGNAL_MS = 2200
// const SINGULARITY_SIGNAL_MS = 2500
// const MAX_WAYPOINTS = 2
// const REFERENCE_FRAME_OPTIONS = [
//   { value: 'Base', label: 'Base' },
//   { value: 'Flange', label: 'Flange' },
//   { value: 'Tool A', label: 'Tool A' },
//   { value: 'Start', label: 'Start' },
//   { value: 'Target', label: 'Target' },
// ]
// const JOG_FRAME_OPTIONS = [
//   { value: 'Base', label: 'Base' },
//   { value: 'Tool', label: 'Tool' },
// ]

// function isPointFilled(point) {
//   return point.x && point.y && point.z && point.rx
// }

// export default function AssemblyModelPage({ onGoExecution }) {
//   const hardware = useHardwareStore()
//   const [mode, setMode] = useState('pick')
//   const [grab, setGrab] = useState(EMPTY_POINT)
//   const [grabFrame, setGrabFrame] = useState('Base')
//   const [drop, setDrop] = useState(EMPTY_POINT)
//   const [dropFrame, setDropFrame] = useState('Base')
//   const [waypoints, setWaypoints] = useState([])
//   const [nextId, setNextId] = useState(1)
//   const [showSuccessModal, setShowSuccessModal] = useState(false)
//   const [isRunningPreview, setIsRunningPreview] = useState(false)
//   const [showCollisionToast, setShowCollisionToast] = useState(false)
//   const [showWrongAnswerToast, setShowWrongAnswerToast] = useState(false)
//   const [hasCollision, setHasCollision] = useState(false)
//   const [showCollisionHintModal, setShowCollisionHintModal] = useState(false)
//   const [selectedCollisionOption, setSelectedCollisionOption] = useState(null)
//   const [collisionHintType, setCollisionHintType] = useState('waypoint')
//   const [collisionHintStep, setCollisionHintStep] = useState(1)
//   const [showRelativeHintInfo, setShowRelativeHintInfo] = useState(false)
//   const [hasTriggeredDirectionCollision, setHasTriggeredDirectionCollision] = useState(false)
//   const [hasTriggeredSingularityCollision, setHasTriggeredSingularityCollision] =
//     useState(false)
//   const [hasSingularityWarning, setHasSingularityWarning] = useState(false)
//   const [isAutomaticReassemblyReady, setIsAutomaticReassemblyReady] = useState(false)
//   const [stage, setStage] = useState('first-block') // first-block | second-block | third-block
//   const [jogFrame, setJogFrame] = useState('Base')
//   const runCompleteTimerRef = useRef(null)
//   const collisionSignalTimerRef = useRef(null)
//   const waitingP2HardwareSignalRef = useRef(false)
//   const waitingDirectionHardwareSignalRef = useRef(false)
//   const waitingSingularityHardwareSignalRef = useRef(false)
//   const stageRef = useRef(stage)
//   const waypointCountRef = useRef(waypoints.length)

//   const canConfirm = isPointFilled(grab) && isPointFilled(drop)
//   const requiresRecordedPoints = stage !== 'first-block'
//   const canConfirmNow = requiresRecordedPoints ? canConfirm : true
//   const connectionInfo = `${
//     hardware.connection === 'connected'
//       ? 'Connected'
//       : hardware.connection === 'error'
//         ? 'Error'
//         : 'Disconnected'
//   } · ${hardware.source === 'hardware' ? 'Real' : 'Virtual'}`

//   useEffect(() => {
//     stageRef.current = stage
//   }, [stage])

//   useEffect(() => {
//     waypointCountRef.current = waypoints.length
//   }, [waypoints.length])

//   useEffect(() => {
//     const cleanupHardware = initializeHardwareStore()
//     const unsubscribeSignal = subscribeHardwareSignal((signal) => {
//       if (
//         waitingP2HardwareSignalRef.current &&
//         signal === HARDWARE_SIGNALS.ASSEMBLY_REACHED_SPECIFIED_POINT &&
//         stageRef.current === 'second-block' &&
//         waypointCountRef.current < 1
//       ) {
//         waitingP2HardwareSignalRef.current = false
//         waitingDirectionHardwareSignalRef.current = false
//         if (runCompleteTimerRef.current !== null) {
//           window.clearTimeout(runCompleteTimerRef.current)
//           runCompleteTimerRef.current = null
//         }
//         if (collisionSignalTimerRef.current !== null) {
//           window.clearTimeout(collisionSignalTimerRef.current)
//           collisionSignalTimerRef.current = null
//         }
//         triggerCollision('waypoint')
//         return
//       }

//       if (
//         waitingDirectionHardwareSignalRef.current &&
//         signal === HARDWARE_SIGNALS.ASSEMBLY_ESTOP_BEFORE_TARGET &&
//         stageRef.current === 'second-block'
//       ) {
//         waitingDirectionHardwareSignalRef.current = false
//         waitingP2HardwareSignalRef.current = false
//         if (runCompleteTimerRef.current !== null) {
//           window.clearTimeout(runCompleteTimerRef.current)
//           runCompleteTimerRef.current = null
//         }
//         if (collisionSignalTimerRef.current !== null) {
//           window.clearTimeout(collisionSignalTimerRef.current)
//           collisionSignalTimerRef.current = null
//         }
//         setHasTriggeredDirectionCollision(true)
//         triggerCollision('direction')
//         return
//       }

//       if (
//         waitingSingularityHardwareSignalRef.current &&
//         signal === HARDWARE_SIGNALS.ASSEMBLY_SINGULARITY_REACHED &&
//         stageRef.current === 'third-block'
//       ) {
//         waitingSingularityHardwareSignalRef.current = false
//         waitingP2HardwareSignalRef.current = false
//         waitingDirectionHardwareSignalRef.current = false
//         if (runCompleteTimerRef.current !== null) {
//           window.clearTimeout(runCompleteTimerRef.current)
//           runCompleteTimerRef.current = null
//         }
//         if (collisionSignalTimerRef.current !== null) {
//           window.clearTimeout(collisionSignalTimerRef.current)
//           collisionSignalTimerRef.current = null
//         }
//         setHasTriggeredSingularityCollision(true)
//         setHasSingularityWarning(true)
//         triggerCollision('singularity')
//       }
//     })

//     return () => {
//       cleanupHardware()
//       unsubscribeSignal()
//       waitingP2HardwareSignalRef.current = false
//       waitingDirectionHardwareSignalRef.current = false
//       waitingSingularityHardwareSignalRef.current = false
//       if (runCompleteTimerRef.current !== null) {
//         window.clearTimeout(runCompleteTimerRef.current)
//       }
//       if (collisionSignalTimerRef.current !== null) {
//         window.clearTimeout(collisionSignalTimerRef.current)
//       }
//       setIsRunningPreview(false)
//     }
//   }, [])

//   useEffect(() => {
//     if (!showCollisionToast) return undefined
//     const toastTimerId = window.setTimeout(() => {
//       setShowCollisionToast(false)
//     }, 3500)
//     return () => window.clearTimeout(toastTimerId)
//   }, [showCollisionToast])

//   useEffect(() => {
//     if (!showWrongAnswerToast) return undefined
//     const toastTimerId = window.setTimeout(() => {
//       setShowWrongAnswerToast(false)
//     }, 2000)
//     return () => window.clearTimeout(toastTimerId)
//   }, [showWrongAnswerToast])

//   const triggerCollision = (type) => {
//     setIsRunningPreview(false)
//     setHasCollision(true)
//     setShowCollisionToast(true)
//     setShowWrongAnswerToast(false)
//     setCollisionHintType(type)
//     setCollisionHintStep(1)
//     setSelectedCollisionOption(null)
//   }

//   const handleAddWaypoint = () => {
//     if (waypoints.length >= MAX_WAYPOINTS) return
//     const currentPoint = captureCurrentPoint()
//     setWaypoints((prev) => [
//       ...prev,
//       {
//         id: nextId,
//         point: currentPoint,
//         frame: jogFrame,
//         isManuallyEdited: false,
//       },
//     ])
//     setNextId((id) => id + 1)
//   }

//   const handleRemoveWaypoint = (id) => {
//     setWaypoints((prev) => prev.filter((waypoint) => waypoint.id !== id))
//   }

//   const handlePointChange = (setter, axis, value) => {
//     setter((prev) => ({ ...prev, [axis]: value }))
//   }

//   const handleWaypointChange = (id, axis, value) => {
//     setWaypoints((prev) =>
//       prev.map((waypoint) =>
//         waypoint.id === id
//           ? {
//               ...waypoint,
//               isManuallyEdited: true,
//               point: {
//                 ...waypoint.point,
//                 [axis]: value,
//               },
//             }
//           : waypoint,
//       ),
//     )
//   }

//   const handleRecordPoint = (setter) => {
//     setter(captureCurrentPoint())
//   }

//   const handleRecordWaypoint = (id) => {
//     const currentPoint = captureCurrentPoint()
//     setWaypoints((prev) =>
//       prev.map((waypoint) =>
//         waypoint.id === id
//           ? {
//               ...waypoint,
//               point: currentPoint,
//               frame: jogFrame,
//               isManuallyEdited: false,
//             }
//           : waypoint,
//       ),
//     )
//   }

//   const handleWaypointFrameChange = (id, frame) => {
//     setWaypoints((prev) =>
//       prev.map((waypoint) =>
//         waypoint.id === id
//           ? {
//               ...waypoint,
//               frame,
//               isManuallyEdited: true,
//             }
//           : waypoint,
//       ),
//     )
//   }

//   const handleConfirmTest = () => {
//     if (isRunningPreview || hardware.isRunning) return
//     if (!canConfirmNow) return
//     waitingP2HardwareSignalRef.current = false
//     waitingDirectionHardwareSignalRef.current = false
//     waitingSingularityHardwareSignalRef.current = false
//     if (hasSingularityWarning) {
//       void resetMockRobotToHome()
//       setHasSingularityWarning(false)
//       setHasCollision(false)
//       setShowCollisionToast(false)
//       setShowWrongAnswerToast(false)
//       setShowCollisionHintModal(false)
//       setSelectedCollisionOption(null)
//       setIsAutomaticReassemblyReady(true)
//       return
//     }

//     setShowSuccessModal(false)
//     setShowCollisionToast(false)
//     setShowWrongAnswerToast(false)
//     setShowCollisionHintModal(false)
//     setSelectedCollisionOption(null)
//     setIsRunningPreview(true)
//     startMockRun(ASSEMBLY_RUN_MS)

//     if (runCompleteTimerRef.current !== null) {
//       window.clearTimeout(runCompleteTimerRef.current)
//     }
//     if (collisionSignalTimerRef.current !== null) {
//       window.clearTimeout(collisionSignalTimerRef.current)
//     }

//     const shouldTriggerWaypointCollision = stage === 'second-block' && waypoints.length < 1
//     const isRealHardwarePath =
//       hardware.source === 'hardware' && hardware.connection === 'connected'
//     const shouldTriggerDirectionCollision = false
//     const shouldTriggerSingularityCollision = false

//     if (shouldTriggerWaypointCollision && isRealHardwarePath) {
//       // P2 rule: in real hardware path, wait for the dedicated hardware signal.
//       waitingP2HardwareSignalRef.current = true
//       return
//     }

//     if (stage === 'second-block') {
//       // Direction error is hardware-signal-driven in block 2.
//       // In mock mode, signal can be injected via window.__ROBOT_DEBUG__.emitSignal(...)
//       waitingDirectionHardwareSignalRef.current = true
//     }

//     if (stage === 'third-block' && !hasTriggeredSingularityCollision) {
//       // Singularity is hardware-signal-driven in block 3.
//       // In mock mode, signal can be injected via window.__ROBOT_DEBUG__.emitSignal(...)
//       waitingSingularityHardwareSignalRef.current = true
//     }

//     if (
//       shouldTriggerWaypointCollision ||
//       shouldTriggerDirectionCollision ||
//       shouldTriggerSingularityCollision
//     ) {
//       collisionSignalTimerRef.current = window.setTimeout(() => {
//         collisionSignalTimerRef.current = null
//         if (shouldTriggerSingularityCollision) {
//           setHasTriggeredSingularityCollision(true)
//           setHasSingularityWarning(true)
//           triggerCollision('singularity')
//           return
//         }
//         if (shouldTriggerDirectionCollision) {
//           setHasTriggeredDirectionCollision(true)
//           triggerCollision('direction')
//           return
//         }
//         triggerCollision('waypoint')
//       }, shouldTriggerSingularityCollision ? SINGULARITY_SIGNAL_MS : COLLISION_SIGNAL_MS)
//       return
//     }

//     runCompleteTimerRef.current = window.setTimeout(() => {
//       runCompleteTimerRef.current = null
//       waitingP2HardwareSignalRef.current = false
//       waitingDirectionHardwareSignalRef.current = false
//       waitingSingularityHardwareSignalRef.current = false
//       setIsRunningPreview(false)
//       setHasCollision(false)
//       setShowSuccessModal(true)
//     }, ASSEMBLY_RUN_MS)
//   }

//   const handleNextBlock = () => {
//     waitingP2HardwareSignalRef.current = false
//     waitingDirectionHardwareSignalRef.current = false
//     waitingSingularityHardwareSignalRef.current = false
//     setShowSuccessModal(false)
//     if (stage === 'first-block') {
//       setStage('second-block')
//     } else if (stage === 'second-block') {
//       setStage('third-block')
//     }
//     setMode('pick')
//     setGrab(EMPTY_POINT)
//     setGrabFrame('Base')
//     setDrop(EMPTY_POINT)
//     setDropFrame('Base')
//     setWaypoints([])
//     setNextId(1)
//     setIsRunningPreview(false)
//     setHasCollision(false)
//     setShowCollisionToast(false)
//     setShowWrongAnswerToast(false)
//     setShowCollisionHintModal(false)
//     setSelectedCollisionOption(null)
//     setCollisionHintType('waypoint')
//     setCollisionHintStep(1)
//     setShowRelativeHintInfo(false)
//     if (stage === 'first-block') {
//       setHasTriggeredDirectionCollision(false)
//     }
//     if (stage === 'second-block') {
//       setHasTriggeredSingularityCollision(false)
//       setHasSingularityWarning(false)
//       setIsAutomaticReassemblyReady(false)
//     }
//   }

//   const handleSuccessPrimaryAction = () => {
//     setShowSuccessModal(false)
//     if (stage === 'third-block' && isAutomaticReassemblyReady) {
//       if (typeof onGoExecution === 'function') {
//         onGoExecution()
//       }
//       return
//     }
//     handleNextBlock()
//   }

//   const handleTryAgainCurrentBlock = () => {
//     waitingP2HardwareSignalRef.current = false
//     waitingDirectionHardwareSignalRef.current = false
//     waitingSingularityHardwareSignalRef.current = false
//     setShowSuccessModal(false)
//     setHasCollision(false)
//     setShowCollisionToast(false)
//     setShowWrongAnswerToast(false)
//     setShowCollisionHintModal(false)
//     setSelectedCollisionOption(null)
//   }

//   const handleJogMove = async (axis, direction) => {
//     const distance = axis === 'rx' ? 5 : 10
//     await sendMockJogMove({
//       axis,
//       direction,
//       frame: jogFrame,
//       distance,
//     })
//   }

//   return (
//     <AssemblyModelPageView
//       stage={stage}
//       mode={mode}
//       grab={grab}
//       grabFrame={grabFrame}
//       drop={drop}
//       dropFrame={dropFrame}
//       waypoints={waypoints}
//       frameOptions={REFERENCE_FRAME_OPTIONS}
//       jogFrameOptions={JOG_FRAME_OPTIONS}
//       canConfirm={canConfirmNow}
//       isAssemblyRunning={isRunningPreview}
//       hasCollision={hasCollision}
//       showCollisionToast={showCollisionToast}
//       showWrongAnswerToast={showWrongAnswerToast}
//       showCollisionHintModal={showCollisionHintModal}
//       selectedCollisionOption={selectedCollisionOption}
//       collisionHintType={collisionHintType}
//       collisionHintStep={collisionHintStep}
//       showRelativeHintInfo={showRelativeHintInfo}
//       showSuccessModal={showSuccessModal}
//       successPrimaryLabel={
//         stage === 'third-block' && isAutomaticReassemblyReady
//           ? 'Automatic reassembly'
//           : 'Next block'
//       }
//       jogFrame={jogFrame}
//       hasSingularityWarning={hasSingularityWarning}
//       connectionInfo={connectionInfo}
//       onToggleMode={() => setMode((prev) => (prev === 'pick' ? 'drop' : 'pick'))}
//       onConfirmTest={handleConfirmTest}
//       onNextBlock={handleNextBlock}
//       onSuccessPrimaryAction={handleSuccessPrimaryAction}
//       onTryAgainCurrentBlock={handleTryAgainCurrentBlock}
//       onOpenCollisionHint={() => setShowCollisionHintModal(true)}
//       onCloseCollisionHint={() => {
//         setShowCollisionHintModal(false)
//         setSelectedCollisionOption(null)
//         if (collisionHintType === 'singularity') return
//         setShowRelativeHintInfo(true)
//       }}
//       onSelectCollisionOption={setSelectedCollisionOption}
//       onConfirmCollisionHint={() => {
//         if (collisionHintType === 'singularity') {
//           if (selectedCollisionOption !== 'C') {
//             setShowWrongAnswerToast(true)
//             return
//           }
//           setShowCollisionHintModal(false)
//           setSelectedCollisionOption(null)
//           setHasCollision(false)
//           setShowWrongAnswerToast(false)
//           return
//         }
//         if (collisionHintType === 'direction' && collisionHintStep === 1) {
//           if (selectedCollisionOption !== 'B') {
//             setShowWrongAnswerToast(true)
//             return
//           }
//           setCollisionHintStep(2)
//           setSelectedCollisionOption(null)
//           setShowWrongAnswerToast(false)
//           return
//         }
//         if (collisionHintType === 'direction' && collisionHintStep === 2) {
//           if (selectedCollisionOption !== 'A') {
//             setShowWrongAnswerToast(true)
//             return
//           }
//           setShowCollisionHintModal(false)
//           setHasCollision(false)
//           setSelectedCollisionOption(null)
//           setShowRelativeHintInfo(true)
//           setShowWrongAnswerToast(false)
//           return
//         }
//         if (selectedCollisionOption !== 'B') {
//           setShowWrongAnswerToast(true)
//           return
//         }
//         setShowCollisionHintModal(false)
//         setHasCollision(false)
//         setSelectedCollisionOption(null)
//         setShowRelativeHintInfo(true)
//         setShowWrongAnswerToast(false)
//       }}
//       onAddWaypoint={handleAddWaypoint}
//       onRemoveWaypoint={handleRemoveWaypoint}
//       onChangeGrabAxis={(axis, value) => handlePointChange(setGrab, axis, value)}
//       onChangeGrabFrame={setGrabFrame}
//       onChangeDropAxis={(axis, value) => handlePointChange(setDrop, axis, value)}
//       onChangeDropFrame={setDropFrame}
//       onChangeWaypointAxis={handleWaypointChange}
//       onChangeWaypointFrame={handleWaypointFrameChange}
//       onChangeJogFrame={setJogFrame}
//       onJogMove={handleJogMove}
//       onRecordGrab={() => handleRecordPoint(setGrab)}
//       onRecordDrop={() => handleRecordPoint(setDrop)}
//       onRecordWaypoint={handleRecordWaypoint}
//       showAddWaypoint={waypoints.length < MAX_WAYPOINTS}
//     />
//   )
// }







import React, { useEffect, useRef, useState } from 'react'
import {
  HARDWARE_SIGNALS,
  captureCurrentPoint,
  initializeHardwareStore,
  resetMockRobotToHome,
  sendMockJogMove,
  subscribeHardwareSignal,
  startMockRun,
  useHardwareStore,
} from '../../services/useHardwareStore.ts'
import { AssemblyModelPageView } from './AssemblyModelPageView.jsx'

const EMPTY_POINT = { x: '', y: '', z: '', rx: '' }
const ASSEMBLY_RUN_MS = 5000 
const COLLISION_SIGNAL_MS = 2200
const SINGULARITY_SIGNAL_MS = 2500
const MAX_WAYPOINTS = 2
const REFERENCE_FRAME_OPTIONS = [
  { value: 'Base', label: 'Base' },
  { value: 'Target', label: 'Target' },
]
const JOG_FRAME_OPTIONS = [
  { value: 'Base', label: 'Base' },
  { value: 'Tool', label: 'Tool' },
]

function isPointFilled(point) {
  return point.x && point.y && point.z && point.rx
}

export default function AssemblyModelPage({ onGoExecution }) {
  const hardware = useHardwareStore()
  const [mode, setMode] = useState('pick')
  const [grab, setGrab] = useState(EMPTY_POINT)
  const [grabFrame, setGrabFrame] = useState('Base')
  const [drop, setDrop] = useState(EMPTY_POINT)
  const [dropFrame, setDropFrame] = useState('Base')
  const [waypoints, setWaypoints] = useState([])
  const [nextId, setNextId] = useState(1)
  
  const [referenceFrame, setReferenceFrame] = useState('Base')
  const [jogFrame, setJogFrame] = useState('Base')

  const [showSuccessModal, setShowSuccessModal] = useState(false)
  const [isRunningPreview, setIsRunningPreview] = useState(false)
  const [showCollisionToast, setShowCollisionToast] = useState(false)
  const [showWrongAnswerToast, setShowWrongAnswerToast] = useState(false)
  const [hasCollision, setHasCollision] = useState(false)
  const [showCollisionHintModal, setShowCollisionHintModal] = useState(false)
  const [selectedCollisionOption, setSelectedCollisionOption] = useState(null)
  const [collisionHintType, setCollisionHintType] = useState('waypoint')
  const [collisionHintStep, setCollisionHintStep] = useState(1)
  const [showRelativeHintInfo, setShowRelativeHintInfo] = useState(false)
  const [hasTriggeredDirectionCollision, setHasTriggeredDirectionCollision] = useState(false)
  const [hasTriggeredSingularityCollision, setHasTriggeredSingularityCollision] = useState(false)
  const [hasSingularityWarning, setHasSingularityWarning] = useState(false)
  const [isAutomaticReassemblyReady, setIsAutomaticReassemblyReady] = useState(false)
  const [stage, setStage] = useState('first-block') 

  const runCompleteTimerRef = useRef(null)
  const collisionSignalTimerRef = useRef(null)
  const waitingP2HardwareSignalRef = useRef(false)
  const waitingDirectionHardwareSignalRef = useRef(false)
  const waitingSingularityHardwareSignalRef = useRef(false)
  const stageRef = useRef(stage)
  const waypointCountRef = useRef(waypoints.length)
  const initCmdSentRef = useRef(false) // 新增：用于防抖和重试的标志位

  const canConfirm = isPointFilled(grab) && isPointFilled(drop)
  const requiresRecordedPoints = stage !== 'first-block'
  const canConfirmNow = requiresRecordedPoints ? canConfirm : true
  const connectionInfo = `${
    hardware.connection === 'connected'
      ? 'Connected'
      : hardware.connection === 'error'
        ? 'Error'
        : 'Disconnected'
  } · ${hardware.source === 'hardware' ? 'Real' : 'Virtual'}`

  // 同步 Refs
  useEffect(() => { stageRef.current = stage }, [stage])
  useEffect(() => { waypointCountRef.current = waypoints.length }, [waypoints.length])

  // ==========================================
  // 🔌 1. 硬件初始化与底层守护指令 (已彻底修复异步丢包Bug)
  // ==========================================
  useEffect(() => {
    const cleanupHardware = initializeHardwareStore()
    initCmdSentRef.current = false

    // 核心修复：解决 WebSocket 还没连上时，指令被吞掉导致不卸力和一直 DISCONNECTED 的问题。
    // 轮询机制：每隔 400ms 尝试下发一次初始化指令（卸力+开启坐标流），直到真实硬件连上为止。
    const initTimer = window.setInterval(() => {
      if (!initCmdSentRef.current && hardware.startAssemblyTeachMode) {
        hardware.startAssemblyTeachMode()
      }
    }, 400)

    const unsubscribeSignal = subscribeHardwareSignal((signal) => {
      if (waitingP2HardwareSignalRef.current && signal === HARDWARE_SIGNALS.ASSEMBLY_REACHED_SPECIFIED_POINT && stageRef.current === 'second-block' && waypointCountRef.current < 1) {
        clearTimers()
        triggerCollision('waypoint')
        return
      }
      if (waitingDirectionHardwareSignalRef.current && signal === HARDWARE_SIGNALS.ASSEMBLY_ESTOP_BEFORE_TARGET && stageRef.current === 'second-block') {
        clearTimers()
        setHasTriggeredDirectionCollision(true)
        triggerCollision('direction')
        return
      }
      if (waitingSingularityHardwareSignalRef.current && signal === HARDWARE_SIGNALS.ASSEMBLY_SINGULARITY_REACHED && stageRef.current === 'third-block') {
        clearTimers()
        setHasTriggeredSingularityCollision(true)
        setHasSingularityWarning(true)
        triggerCollision('singularity')
      }
    })

    return () => {
      window.clearInterval(initTimer)
      cleanupHardware()
      unsubscribeSignal()
      clearTimers()
      setIsRunningPreview(false)
    }
  }, [])

  // 监听到真实硬件连上并开始回传坐标后，停止轮询，防止重复发 'K' 导致坐标流闪烁
  useEffect(() => {
    if (hardware.connection === 'connected' && hardware.source === 'hardware') {
      initCmdSentRef.current = true
    }
  }, [hardware.connection, hardware.source])

  // 坐标系和电磁铁的实时控制（仅真机生效）
  useEffect(() => {
    if (hardware.setReferenceFrame) hardware.setReferenceFrame(referenceFrame)
  }, [referenceFrame])

  useEffect(() => {
    if (hardware.controlMagnet) hardware.controlMagnet(mode === 'pick')
  }, [mode])

  // ==========================================
  // 🎮 2. Mock 模式与交互逻辑恢复
  // ==========================================
  const clearTimers = () => {
    waitingP2HardwareSignalRef.current = false
    waitingDirectionHardwareSignalRef.current = false
    waitingSingularityHardwareSignalRef.current = false
    if (runCompleteTimerRef.current !== null) window.clearTimeout(runCompleteTimerRef.current)
    if (collisionSignalTimerRef.current !== null) window.clearTimeout(collisionSignalTimerRef.current)
    runCompleteTimerRef.current = null
    collisionSignalTimerRef.current = null
  }

  const triggerCollision = (type) => {
    setIsRunningPreview(false)
    setHasCollision(true)
    setShowCollisionToast(true)
    setShowWrongAnswerToast(false)
    setCollisionHintType(type)
    setCollisionHintStep(1)
    setSelectedCollisionOption(null)
  }

  const handlePointChange = (setter, axis, value) => {
    setter((prev) => ({ ...prev, [axis]: value }))
  }

  const handleWaypointChange = (id, axis, value) => {
    setWaypoints((prev) => prev.map((wp) => wp.id === id ? { ...wp, isManuallyEdited: true, point: { ...wp.point, [axis]: value } } : wp))
  }

  const handleJogMove = async (axis, direction) => {
    const distance = axis === 'rx' ? 5 : 10
    if (hardware.sendMockJogMove) {
      await hardware.sendMockJogMove({ axis, direction, frame: jogFrame, distance })
    }
  }

  const handleRecordPoint = (setter) => setter(hardware.captureCurrentPoint())
  const handleRecordWaypoint = (id) => {
    const currentPoint = hardware.captureCurrentPoint()
    setWaypoints((prev) => prev.map((wp) => wp.id === id ? { ...wp, point: currentPoint, frame: jogFrame, isManuallyEdited: false } : wp))
  }

  const handleAddWaypoint = () => {
    if (waypoints.length >= MAX_WAYPOINTS) return
    setWaypoints((prev) => [...prev, { id: nextId, point: EMPTY_POINT, frame: jogFrame, isManuallyEdited: false }])
    setNextId((id) => id + 1)
  }
  const handleRemoveWaypoint = (id) => setWaypoints((prev) => prev.filter((wp) => wp.id !== id))

  // ==========================================
  // 🚀 3. CONFIRM & TEST 双轨执行引擎
  // ==========================================
  const handleConfirmTest = () => {
    if (isRunningPreview || hardware.isRunning) return
    if (!canConfirmNow) return
    
    clearTimers()

    if (hasSingularityWarning) {
      if (hardware.resetMockRobotToHome) hardware.resetMockRobotToHome()
      setHasSingularityWarning(false)
      setHasCollision(false)
      setShowCollisionToast(false)
      setShowWrongAnswerToast(false)
      setShowCollisionHintModal(false)
      setSelectedCollisionOption(null)
      setIsAutomaticReassemblyReady(true)
      return
    }

    setShowSuccessModal(false)
    setShowCollisionToast(false)
    setShowWrongAnswerToast(false)
    setShowCollisionHintModal(false)
    setSelectedCollisionOption(null)
    setIsRunningPreview(true)

    const isRealHardwarePath = hardware.source === 'hardware' && hardware.connection === 'connected'
    
    // 动态计算真实硬件跑完所需的准确时间
    const realHardwareDurationMs = 500 + 2000 + 1500 + 1500 + (waypoints.length * 2000) + 2000 + 1500
    const currentRunDuration = isRealHardwarePath ? realHardwareDurationMs : ASSEMBLY_RUN_MS

    if (isRealHardwarePath) {
      hardware.executeAssemblyPath(grab, waypoints.map(w => w.point), drop, referenceFrame)
    } else {
      if (hardware.startMockRun) hardware.startMockRun(ASSEMBLY_RUN_MS)
    }

    const shouldTriggerWaypointCollision = stage === 'second-block' && waypoints.length < 1
    const shouldTriggerDirectionCollision = false
    const shouldTriggerSingularityCollision = false

    if (shouldTriggerWaypointCollision && isRealHardwarePath) {
      waitingP2HardwareSignalRef.current = true
      return
    }

    if (stage === 'second-block') waitingDirectionHardwareSignalRef.current = true
    if (stage === 'third-block' && !hasTriggeredSingularityCollision) waitingSingularityHardwareSignalRef.current = true

    if (shouldTriggerWaypointCollision || shouldTriggerDirectionCollision || shouldTriggerSingularityCollision) {
      collisionSignalTimerRef.current = window.setTimeout(() => {
        collisionSignalTimerRef.current = null
        if (shouldTriggerSingularityCollision) {
          setHasTriggeredSingularityCollision(true)
          setHasSingularityWarning(true)
          triggerCollision('singularity')
          return
        }
        if (shouldTriggerDirectionCollision) {
          setHasTriggeredDirectionCollision(true)
          triggerCollision('direction')
          return
        }
        triggerCollision('waypoint')
      }, shouldTriggerSingularityCollision ? SINGULARITY_SIGNAL_MS : COLLISION_SIGNAL_MS)
      return
    }

    runCompleteTimerRef.current = window.setTimeout(() => {
      clearTimers()
      setIsRunningPreview(false)
      setHasCollision(false)
      setShowSuccessModal(true)
    }, currentRunDuration)
  }

  const handleNextBlock = () => {
    clearTimers()
    setShowSuccessModal(false)
    if (stage === 'first-block') setStage('second-block')
    else if (stage === 'second-block') setStage('third-block')
    setMode('pick')
    setGrab(EMPTY_POINT); setGrabFrame('Base'); setDrop(EMPTY_POINT); setDropFrame('Base')
    setWaypoints([]); setNextId(1)
    setIsRunningPreview(false); setHasCollision(false)
    setShowCollisionToast(false); setShowWrongAnswerToast(false); setShowCollisionHintModal(false)
    setSelectedCollisionOption(null); setCollisionHintType('waypoint'); setCollisionHintStep(1)
    setShowRelativeHintInfo(false)
    if (stage === 'first-block') setHasTriggeredDirectionCollision(false)
    if (stage === 'second-block') {
      setHasTriggeredSingularityCollision(false); setHasSingularityWarning(false); setIsAutomaticReassemblyReady(false)
    }
  }

  const handleSuccessPrimaryAction = () => {
    setShowSuccessModal(false)
    if (stage === 'third-block' && isAutomaticReassemblyReady) {
      if (typeof onGoExecution === 'function') onGoExecution()
      return
    }
    handleNextBlock()
  }

  const handleTryAgainCurrentBlock = () => {
    clearTimers()
    setShowSuccessModal(false); setHasCollision(false); setShowCollisionToast(false)
    setShowWrongAnswerToast(false); setShowCollisionHintModal(false); setSelectedCollisionOption(null)
  }

  return (
    <AssemblyModelPageView
      stage={stage}
      mode={mode}
      grab={grab}
      grabFrame={grabFrame}
      drop={drop}
      dropFrame={dropFrame}
      waypoints={waypoints}
      frameOptions={REFERENCE_FRAME_OPTIONS}
      jogFrameOptions={JOG_FRAME_OPTIONS}
      canConfirm={canConfirmNow}
      isAssemblyRunning={isRunningPreview || hardware.isRunning}
      hasCollision={hasCollision}
      showCollisionToast={showCollisionToast}
      showWrongAnswerToast={showWrongAnswerToast}
      showCollisionHintModal={showCollisionHintModal}
      selectedCollisionOption={selectedCollisionOption}
      collisionHintType={collisionHintType}
      collisionHintStep={collisionHintStep}
      showRelativeHintInfo={showRelativeHintInfo}
      showSuccessModal={showSuccessModal}
      successPrimaryLabel={stage === 'third-block' && isAutomaticReassemblyReady ? 'Automatic reassembly' : 'Next block'}
      jogFrame={jogFrame}
      hasSingularityWarning={hasSingularityWarning}
      connectionInfo={connectionInfo}
      
      onToggleMode={() => setMode((prev) => (prev === 'pick' ? 'drop' : 'pick'))}
      onConfirmTest={handleConfirmTest}
      onNextBlock={handleNextBlock}
      onSuccessPrimaryAction={handleSuccessPrimaryAction}
      onTryAgainCurrentBlock={handleTryAgainCurrentBlock}
      
      onOpenCollisionHint={() => setShowCollisionHintModal(true)}
      onCloseCollisionHint={() => {
        setShowCollisionHintModal(false)
        setSelectedCollisionOption(null)
        if (collisionHintType === 'singularity') return
        setShowRelativeHintInfo(true)
      }}
      onSelectCollisionOption={setSelectedCollisionOption}
      onConfirmCollisionHint={() => {
        if (collisionHintType === 'singularity') {
          if (selectedCollisionOption !== 'C') { setShowWrongAnswerToast(true); return }
          setShowCollisionHintModal(false); setSelectedCollisionOption(null); setHasCollision(false); setShowWrongAnswerToast(false); return
        }
        if (collisionHintType === 'direction' && collisionHintStep === 1) {
          if (selectedCollisionOption !== 'B') { setShowWrongAnswerToast(true); return }
          setCollisionHintStep(2); setSelectedCollisionOption(null); setShowWrongAnswerToast(false); return
        }
        if (collisionHintType === 'direction' && collisionHintStep === 2) {
          if (selectedCollisionOption !== 'A') { setShowWrongAnswerToast(true); return }
          setShowCollisionHintModal(false); setHasCollision(false); setSelectedCollisionOption(null); setShowRelativeHintInfo(true); setShowWrongAnswerToast(false); return
        }
        if (selectedCollisionOption !== 'B') { setShowWrongAnswerToast(true); return }
        setShowCollisionHintModal(false); setHasCollision(false); setSelectedCollisionOption(null); setShowRelativeHintInfo(true); setShowWrongAnswerToast(false)
      }}
      
      onAddWaypoint={handleAddWaypoint}
      onRemoveWaypoint={handleRemoveWaypoint}
      
      onChangeGrabAxis={(axis, value) => handlePointChange(setGrab, axis, value)}
      onChangeGrabFrame={setReferenceFrame}
      onChangeDropAxis={(axis, value) => handlePointChange(setDrop, axis, value)}
      onChangeDropFrame={setReferenceFrame}
      onChangeWaypointAxis={handleWaypointChange}
      onChangeWaypointFrame={(id, val) => setWaypoints((p) => p.map(w => w.id === id ? { ...w, frame: val } : w))}
      
      onChangeJogFrame={setJogFrame}
      onJogMove={handleJogMove}
      
      onRecordGrab={() => handleRecordPoint(setGrab)}
      onRecordDrop={() => handleRecordPoint(setDrop)}
      onRecordWaypoint={handleRecordWaypoint}
      showAddWaypoint={waypoints.length < MAX_WAYPOINTS}
    />
  )
}
