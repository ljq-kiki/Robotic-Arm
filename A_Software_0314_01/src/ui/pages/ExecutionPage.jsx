import React, { useEffect, useRef, useState } from 'react'
import { PageLayout } from '../components/PageLayout.jsx'
import { PixelCard } from '../components/PixelCard.jsx'
import { PixelButton } from '../components/PixelButton.jsx'
import { StepBar } from '../components/StepBar.jsx'
// import { initializeHardwareStore, useHardwareStore } from '../../services/useHardwareStore.ts'
//0405
import { 
  initializeHardwareStore, 
  useHardwareStore, 
  subscribeHardwareSignal, 
  HARDWARE_SIGNALS 
} from '../../services/useHardwareStore.ts'
import { mediaAssets } from '../mediaAssets.js'
import { CelebrationImage } from '../components/CelebrationImage.jsx'
import { ResetArmButton } from '../components/ResetArmButton.jsx'
import { ConnectionStatusLabel } from '../components/ConnectionStatusLabel.jsx'
import './ExecutionPage.css'

const steps = [
  { id: 'step-1', status: 'done', position: 'first', label: '', title: 'INSTALL' },
  { id: 'step-2', status: 'done', position: 'middle', label: '', title: 'TEST' },
  { id: 'step-3', status: 'done', position: 'middle', label: '', title: 'ASSEMBLY' },
  { id: 'step-4', status: 'active', position: 'last', label: '', title: 'EXECUTE' },
]

const EXECUTION_SUCCESS_MS = 5200
const DANGER_SIGNAL_MS = 2300

function getInitialPhase() {
  if (import.meta.env.DEV && typeof window !== 'undefined') {
    const q = new URLSearchParams(window.location.search)
    if (q.get('preview') === 'execution-success') return 'success'
  }
  return 'idle'
}

export default function ExecutionPage({ onRestartGame }) {
  const hardware = useHardwareStore()
  const connectionInfo = `${
    hardware.connection === 'connected'
      ? 'Connected'
      : hardware.connection === 'error'
        ? 'Error'
        : 'Disconnected'
  } ?? ${hardware.source === 'hardware' ? 'Real' : 'Virtual'}`
  const [phase, setPhase] = useState(getInitialPhase) // idle | running | paused-danger | success
  const [progress, setProgress] = useState(0)
  const [dangerWarning, setDangerWarning] = useState(false)
  const [hasTriggeredDangerSignal, setHasTriggeredDangerSignal] = useState(false)

  const progressIntervalRef = useRef(null)
  const completeTimeoutRef = useRef(null)
  const dangerTimeoutRef = useRef(null)

  const clearRunTimers = () => {
    if (progressIntervalRef.current !== null) {
      window.clearInterval(progressIntervalRef.current)
      progressIntervalRef.current = null
    }
    if (completeTimeoutRef.current !== null) {
      window.clearTimeout(completeTimeoutRef.current)
      completeTimeoutRef.current = null
    }
    if (dangerTimeoutRef.current !== null) {
      window.clearTimeout(dangerTimeoutRef.current)
      dangerTimeoutRef.current = null
    }
  }

  useEffect(() => clearRunTimers, [])
  // useEffect(() => {
  //   const cleanupHardware = initializeHardwareStore()
  //   return cleanupHardware
  // }, [])

  //0405
  const waitingExecutionFinishRef = useRef(false)
  //0405
  useEffect(() => {
    const cleanupHardware = initializeHardwareStore()
    
    // === ?????????????????????????????? ===
    const unsubscribeSignal = subscribeHardwareSignal((rawSignal) => {
      if (
        waitingExecutionFinishRef.current && 
        rawSignal === HARDWARE_SIGNALS.EXECUTION_RUN_FINISHED
      ) {
        waitingExecutionFinishRef.current = false
        clearRunTimers()
        setProgress(1) // ?????????????
        setPhase('success') // ??????????
        setDangerWarning(false)
      }

        //0406 === ??????????2. ??????????????????????? UI ???????? ===
      if (
        waitingExecutionFinishRef.current && 
        rawSignal === 'RAW_ESTOP_TRIGGERED'
      ) {
        waitingExecutionFinishRef.current = false
        clearRunTimers() // ????? 90 ??????????????????
        setProgress(0)  // ????????
        setPhase('idle') // ????????????????"Run the Program" ????????????
        setDangerWarning(true) // ??????????????????????
      }
    })
    
    return () => {
      cleanupHardware()
      unsubscribeSignal()
    }
  }, [])

  // const startExecutionRun = () => {
  //   clearRunTimers()
  //   setPhase('running')
  //   setDangerWarning(false)
  //   setProgress(0)

  //   const startedAt = Date.now()
  //   progressIntervalRef.current = window.setInterval(() => {
  //     const elapsed = Date.now() - startedAt
  //     setProgress(Math.min(0.98, elapsed / EXECUTION_SUCCESS_MS))
  //   }, 120)

  //   if (!hasTriggeredDangerSignal) {
  //     dangerTimeoutRef.current = window.setTimeout(() => {
  //       dangerTimeoutRef.current = null
  //       clearRunTimers()
  //       setHasTriggeredDangerSignal(true)
  //       setPhase('paused-danger')
  //       setDangerWarning(true)
  //       setProgress(0)
  //     }, DANGER_SIGNAL_MS)
  //     return
  //   }

  //   completeTimeoutRef.current = window.setTimeout(() => {
  //     completeTimeoutRef.current = null
  //     clearRunTimers()
  //     setProgress(1)
  //     setPhase('success')
  //     setDangerWarning(false)
  //   }, EXECUTION_SUCCESS_MS)
  // }

  const startExecutionRun = () => {
    clearRunTimers()
    setPhase('running')
    setDangerWarning(false)
    setProgress(0)

    // 1. ???? D ?????????????????????
    hardware.startExecutionBatch()
    waitingExecutionFinishRef.current = true

    // 2. ??????????????? (??????????? 98% ?????????)
    const startedAt = Date.now()
    progressIntervalRef.current = window.setInterval(() => {
      const elapsed = Date.now() - startedAt
      // ???????????????????? 35 ???????????????
      setProgress(Math.min(0.98, elapsed / 35000))
    }, 120)

    // 3. ????????????? (?????????? 5.2 ??????????? 2.3 ??????)
    completeTimeoutRef.current = window.setTimeout(() => {
      if (waitingExecutionFinishRef.current) {
         waitingExecutionFinishRef.current = false
         clearRunTimers()
         setProgress(1)
         setPhase('success')
         setDangerWarning(false)
      }
    }, 90000) // 90???????
  }

  if (phase === 'success') {
    return (
      <PageLayout>
        <div className="mb-10 flex shrink-0 items-center justify-between">
          <div className="assembly-page-title px text-[24px]">Lion Model Assembly Game</div>
          <div className="flex items-center gap-3">
            <ConnectionStatusLabel text={connectionInfo} />
            <ResetArmButton />
            <div className="swatch" style={{ background: 'var(--bgPurple)' }} />
            <div className="swatch" style={{ background: 'var(--orange)' }} />
            <div className="swatch" style={{ background: 'var(--magenta)' }} />
          </div>
        </div>
        <div className="execution-success-screen">
          <div className="execution-success-card">
            <div className="execution-success-panel">
              <div className="execution-success-copy">
                <div className="execution-success-title px">Congratulations!</div>
              </div>
              <CelebrationImage
                frameClassName="execution-success-dashed-frame"
                imageClassName="execution-success-celebration-gif"
                alt=""
                adaptAspect
              />
            </div>
          </div>
          <PixelButton
            variant="magenta"
            className="execution-success-cta"
            onClick={onRestartGame}
          >
            Restart the game
          </PixelButton>
        </div>
      </PageLayout>
    )
  }

  return (
    <PageLayout>
      <div className="flex items-center justify-between shrink-0 mb-10">
        <div className="assembly-page-title px text-[24px]">Lion Model Assembly Game</div>
        <div className="flex items-center gap-3">
          <ConnectionStatusLabel text={connectionInfo} />
          <ResetArmButton />
          <div className="swatch" style={{ background: 'var(--bgPurple)' }} />
          <div className="swatch" style={{ background: 'var(--orange)' }} />
          <div className="swatch" style={{ background: 'var(--magenta)' }} />
        </div>
      </div>

      <StepBar steps={steps} />

      <div className="grid max-lg:grid-cols-1 max-lg:[grid-auto-rows:max-content] max-lg:flex-none gap-8 lg:grid-cols-[4fr_3fr] lg:flex-1 lg:min-h-0">
        <PixelCard
          padding="p-6"
          className="flex min-h-0 max-h-full flex-col overflow-hidden max-lg:max-h-none"
        >
          <div className="flex min-h-0 flex-1 flex-col gap-5 overflow-y-auto max-lg:min-h-min max-lg:flex-none max-lg:overflow-visible">
            <div className="execution-title px text-[24px]">Automated Block Assembly</div>
            <div className="execution-paragraph-row">
              <span className="execution-inline-icon execution-inline-icon-brick" aria-hidden="true">
                <img
                  src={mediaAssets.blockBrickIcon}
                  alt=""
                  className="execution-inline-icon-image"
                />
              </span>
              <div className="execution-paragraph text-[18px] leading-[1.35]">
                <span className="execution-emphasis">Reset the blocks</span>, then run
                the robot in automatic mode to finish the full assembly.
              </div>
            </div>
            <div className="execution-paragraph-row">
              <span className="execution-inline-icon" aria-hidden="true">
                <img
                  src={mediaAssets.emergencyStopIcon}
                  alt=""
                  className="execution-inline-icon-image"
                />
              </span>
              <div className="execution-paragraph text-[18px] leading-[1.35]">
                <span className="execution-emphasis">Hold the E-stop button</span> while
                running, and <span className="execution-emphasis">stay away</span> from
                the danger area.
              </div>
            </div>

            <div className="execution-decoration-box" aria-hidden="true">
              <img
                src={mediaAssets.emergencyStopIcon}
                alt=""
                className="execution-decoration-image"
              />
            </div>

          </div>

          <div className="mt-4 flex items-center gap-4">
            {phase === 'running' ? (
              <div className="assembly-running-shell w-full">
                <div className="running-bar assembly-running-fill" />
                <span className="assembly-running-label px text-[13px]">
                  Running...
                </span>
              </div>
            ) : (
              <PixelButton
                variant="magenta"
                className="w-full py-4 text-[18px]"
                onClick={startExecutionRun}
              >
                Run the Program
              </PixelButton>
            )}
          </div>

        </PixelCard>

        <PixelCard
          title="Robot Workspace"
          titleColor="var(--orange)"
          className="flex min-h-0 max-h-full flex-col overflow-hidden max-lg:max-h-none"
        >
          <div className="assembly-model-stage execution-model-card-stage execution-workspace-stage flex min-h-0 flex-1 flex-col overflow-hidden max-lg:min-h-[min(52vh,340px)] max-lg:flex-none">
            <div className="execution-model-dashed-inner">
              <img
                src={mediaAssets.robotWorkspaceMap}
                alt="Robot workspace map"
                className="execution-card-model-image"
              />
            </div>
            {dangerWarning && (
              <div className="execution-danger-warning px py-2 text-center shrink-0">
                Don&apos;t step in the dangerous area
              </div>
            )}
          </div>
        </PixelCard>
      </div>

    </PageLayout>
  )
}
