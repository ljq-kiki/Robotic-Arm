import React, { useState } from 'react'
import InstallToolPage from './InstallToolPage.jsx'
import InstallCalibrationPage from './InstallCalibrationPage.jsx'
import TestToolPage from './TestToolPage.jsx'
import AssemblyModelPage from './AssemblyModelPage.jsx'
import ExecutionPage from './ExecutionPage.jsx'

/**
 * 开发预览：
 * - ?preview=execution — 直接进入 Execute 主界面（idle）
 * - ?preview=execution-success — 同上，且 ExecutionPage 进入庆祝完成态
 * - ?preview=assembly-singularity — 直接进入 Assembly 奇异点错误预览
 * - ?preview=assembly-estop — 直接进入 Assembly E-stop 错误预览
 * - ?preview=assembly-direction — 直接进入 Assembly 坐标系错误预览
 */
function getInitialStep() {
  if (import.meta.env.DEV && typeof window !== 'undefined') {
    const q = new URLSearchParams(window.location.search)
    const p = q.get('preview')
    if (p === 'assembly-singularity' || p === 'assembly-estop' || p === 'assembly-direction') return 'assembly'
    if (p === 'execution' || p === 'execution-success') return 'execution'
  }
  return 'install'
}

export default function InstallFlowPage() {
  const [step, setStep] = useState(getInitialStep)
  const [calibrationResult, setCalibrationResult] = useState({
    calibratedPayload: '0.5kg',
  })

  if (step === 'install') {
    return <InstallToolPage onInstalled={() => setStep('calibration')} />
  }

  if (step === 'calibration') {
    return (
      <InstallCalibrationPage
        onNext={(result) => {
          if (result?.calibratedPayload) {
            setCalibrationResult({
              calibratedPayload: result.calibratedPayload,
            })
          }
          setStep('test')
        }}
      />
    )
  }

  if (step === 'test') {
    return (
      <TestToolPage
        calibratedPayload={calibrationResult.calibratedPayload}
        onStartGame={() => setStep('assembly')}
      />
    )
  }

  if (step === 'assembly') {
    return <AssemblyModelPage onGoExecution={() => setStep('execution')} />
  }

  if (step === 'execution') {
    return <ExecutionPage onRestartGame={() => setStep('install')} />
  }

  return null
}

