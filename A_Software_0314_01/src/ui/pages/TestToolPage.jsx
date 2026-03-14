import React, { useState, useEffect } from 'react'
import { PageLayout } from '../components/PageLayout.jsx'
import { PixelCard } from '../components/PixelCard.jsx'
import { PixelButton } from '../components/PixelButton.jsx'
import { StepBar } from '../components/StepBar.jsx'
import { PixelSelect } from '../components/PixelSelect.jsx'
import { PixelInput } from '../components/PixelInput.jsx'
import { PixelToast } from '../components/PixelToast.jsx'
import { PixelRadio } from '../components/PixelRadio.jsx'

const steps = [
  {
    id: 'step-1',
    status: 'done',
    position: 'first',
    label: '',
    title: 'INSTALL',
  },
  {
    id: 'step-2',
    status: 'active',
    position: 'middle',
    label: '',
    title: 'TEST',
  },
  {
    id: 'step-3',
    status: 'pending_3',
    position: 'middle',
    label: '',
    title: 'ASSEMBLY',
  },
  {
    id: 'step-4',
    status: 'pending_4',
    position: 'last',
    label: '',
    title: 'EXECUTE',
  },
]

export default function TestToolPage({ onStartGame }) {
  const [status, setStatus] = useState('idle') // idle | running | error | success
  const [showToast, setShowToast] = useState(false)
  const [showHintModal, setShowHintModal] = useState(false)
  const [selectedOption, setSelectedOption] = useState(null)
  const [selectedTool, setSelectedTool] = useState('Flange')

  const isRunning = status === 'running'
  const hasError = status === 'error'
  const isSuccess = status === 'success'

  const handleTestClick = () => {
    setStatus('running')
    window.setTimeout(() => {
      if (selectedTool === 'Tool 1') {
        setStatus('success')
        setShowToast(false)
      } else {
        setStatus('error')
        setShowToast(true)
      }
    }, 2000)
  }

  useEffect(() => {
    if (!showToast) return
    const id = window.setTimeout(() => {
      setShowToast(false)
    }, 3000)
    return () => window.clearTimeout(id)
  }, [showToast])

  return (
    <PageLayout>
      <div className="flex items-center justify-between shrink-0 mb-10">
        <div className="px text-[24px]" style={{ color: 'var(--panel)' }}>
          Lion Model Assembly Game
        </div>
        <div className="flex gap-3">
          <div className="swatch" style={{ background: 'var(--bgPurple)' }} />
          <div className="swatch" style={{ background: 'var(--orange)' }} />
          <div className="swatch" style={{ background: 'var(--magenta)' }} />
        </div>
      </div>

      <StepBar steps={steps} />

      <PixelToast
        open={hasError && showToast}
        icon="⚠"
        message="Collision occurred! Try to solve it"
      />

      <div className="grid lg:grid-cols-2 gap-8 flex-1 min-h-0">
        <PixelCard
          padding="p-6"
          className="min-h-0 max-h-full flex flex-col overflow-hidden"
        >
          <div className="flex-1 min-h-0 overflow-y-auto flex flex-col gap-4">
            <div
              className="px text-[13px]"
              style={{ color: 'var(--magenta)' }}
            >
              Check your tool, pick up the teddy bear, and complete the tool test to
              begin the game.
            </div>

            <div className="flex flex-col gap-3">
              {/* Tool + Payload 两列布局，每列内部是“标题在上，输入在下” */}
              <div className="grid grid-cols-2 gap-4">
                <PixelSelect
                  className="w-full"
                  label="TOOL"
                  value={selectedTool}
                  onChange={(value) => setSelectedTool(value)}
                  options={[
                    { value: 'Flange', label: 'Flange' },
                    { value: 'Tool 1', label: 'Tool 1' },
                    { value: 'Gripper', label: 'Gripper' },
                    { value: 'Welder', label: 'Welder' },
                  ]}
                />
                <PixelInput
                  label="PAYLOAD"
                  defaultValue="-"
                  className="w-full"
                />
              </div>

              <div className="mt-1">
                <div className="px text-[12px] mb-1">TCP</div>
                <div className="grid grid-cols-4 gap-3">
                  {[
                    { axis: 'X', value: '200.00' },
                    { axis: 'Y', value: '100.00' },
                    { axis: 'Z', value: '200.00' },
                    { axis: 'Rx', value: '0.00' },
                  ].map(({ axis, value }) => (
                    <div key={axis} className="flex items-center gap-1">
                      <div className="px text-[12px]">{axis}</div>
                      <PixelInput
                        className="w-full max-w-[96px]"
                        defaultValue={value}
                      />
                    </div>
                  ))}
                </div>
              </div>
            </div>

            <div
              className="mt-4 pixel-card soft-grid"
              style={{
                background: 'var(--panel)',
                minHeight: '180px',
              }}
            >
              <div className="w-full h-full flex items-center justify-center">
                <div className="text-center">
                  <div className="px text-[12px] mb-2">GIF PLACEHOLDER</div>
                  <div className="text-[10px]" style={{ color: 'var(--muted)' }}>
                    Grab the teddy bear · Highlight the TCP
                  </div>
                </div>
              </div>
            </div>

            <div className="flex-1" />

            <div className="shrink-0 mt-4 w-full">
              {isRunning ? (
                <div
                  className="w-full"
                  style={{
                    borderRadius: '9999px',
                    border: '2px solid var(--ink)',
                    background: 'var(--panel)',
                    overflow: 'hidden',
                    height: '40px',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    position: 'relative',
                  }}
                >
                  <div
                    className="running-bar"
                    style={{
                      height: '100%',
                      width: '60%',
                      background: '#A5FFB5',
                      position: 'absolute',
                      left: 0,
                      top: 0,
                      bottom: 0,
                    }}
                  >
                  </div>
                  <span
                    className="px text-[13px]"
                    style={{ color: 'var(--ink)', position: 'relative', zIndex: 1 }}
                  >
                    Running...
                  </span>
                </div>
              ) : (
                <PixelButton
                  variant="magenta"
                  className="w-full py-4 text-[12px]"
                  onClick={isSuccess && onStartGame ? onStartGame : handleTestClick}
                  icon={isSuccess ? '🎉' : undefined}
                >
                  {isSuccess ? 'Start Game !' : hasError ? 'TEST AGAIN' : 'TEST'}
                </PixelButton>
              )}
            </div>
          </div>
        </PixelCard>

        <PixelCard
          title="3D ROBOT MODEL"
          titleColor="var(--orange)"
          className="min-h-0 max-h-full flex flex-col overflow-hidden"
        >
          <div
            className="flex-1 min-h-0 overflow-y-auto flex items-center justify-center"
            style={{
              border: '3px dashed var(--ink)',
              background: 'var(--panel)',
            }}
          >
            <div className="relative w-full h-full flex items-center justify-center py-16">
              <div className="text-center">
                <div className="px text-[16px] mb-2">3D Robot Model</div>
                <div className="px text-[12px]" style={{ color: 'var(--muted)' }}>
                  With tool
                </div>
              </div>

              {hasError && (
                <button
                  type="button"
                  onClick={() => setShowHintModal(true)}
                  style={{
                    position: 'absolute',
                    right: '24px',
                    bottom: '24px',
                    width: '60px',
                    height: '60px',
                    borderRadius: '9999px',
                    border: '2px solid var(--ink)',
                    boxShadow: '4px 4px 0 var(--shadow)',
                    background: '#FFD5D5',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                  }}
                >
                  <span
                    style={{
                      fontSize: '26px',
                      textShadow: '2px 2px 0 var(--shadow)',
                    }}
                  >
                    💡
                  </span>
                </button>
              )}
            </div>
          </div>
        </PixelCard>
      </div>

      {showHintModal && (
        <div
          className="fixed inset-0 z-50 flex items-center justify-center"
          style={{ background: 'rgba(0,0,0,0.35)' }}
        >
          <div
            className="pixel-card soft-grid p-10"
            style={{
              background: 'var(--panel)',
              maxWidth: '1120px',
              width: 'calc(100% - 64px)',
              maxHeight: '82vh',
              overflowY: 'auto',
            }}
          >
            <div className="mb-8">
              <div
                className="px text-[18px] mb-6 flex items-center"
                style={{ color: '#FF3B3B' }}
              >
                <span
                  style={{
                    display: 'inline-flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    transform: 'translateY(-2px) scale(1.5)',
                    transformOrigin: 'center',
                    marginRight: '10px',
                  }}
                >
                  💡
                </span>
                Collision occurred! Check the tool before you run.
              </div>

              <div className="flex justify-between items-start gap-8">
                <div className="flex-1">
                  <div
                    className="text-[20px] flex flex-col items-center gap-2 text-center"
                    style={{ color: 'var(--ink)' }}
                  >
                    <div>If the tool in the program doesn&apos;t match the real one</div>
                    <div
                      className="px"
                      style={{
                        fontSize: '24px',
                        textShadow: '2px 2px 0 var(--shadow)',
                      }}
                    >
                      ↓
                    </div>
                    <div>The robot will use the wrong TCP while working</div>
                    <div
                      className="px"
                      style={{
                        fontSize: '24px',
                        textShadow: '2px 2px 0 var(--shadow)',
                      }}
                    >
                      ↓
                    </div>
                    <div>
                      This can lead to collisions or missed targets during execution.
                    </div>
                  </div>
                </div>

                <div
                  className="pixel-card soft-grid"
                  style={{
                    width: '440px',
                    height: '320px',
                    background: '#FFFFFF',
                    display: 'flex',
                    padding: '12px',
                  }}
                >
                  <div className="flex flex-col w-full h-full">
                    <div className="flex-1 overflow-hidden flex items-center justify-center mb-2">
                      <img
                        src="/placeholders/chopsticks-illustration.svg"
                        alt="Eating with the chopsticks"
                        style={{
                          width: '100%',
                          height: '100%',
                          objectFit: 'cover',
                        }}
                      />
                    </div>
                    <div
                      className="text-[13px]"
                      style={{ color: 'var(--muted)', lineHeight: 1.2 }}
                    >
                      You practiced eating with long chopsticks. When it&apos;s time to eat,
                      you forget you&apos;re holding them and try to grab food by hand —
                      the chopsticks crash straight into the plate.
                    </div>
                  </div>
                </div>
              </div>
            </div>

            <div className="mt-6">
              <div
                className="mb-4"
                style={{
                  height: '2px',
                  background:
                    'repeating-linear-gradient(90deg, var(--ink) 0 8px, transparent 8px 12px)',
                  boxShadow: '0 3px 0 var(--shadow)',
                }}
              />
              <div className="text-[15px] mb-4" style={{ color: 'var(--ink)' }}>
                <span
                  style={{
                    display: 'inline-block',
                    transform: 'scale(1.5)',
                    transformOrigin: 'center',
                    marginRight: '8px',
                  }}
                >
                  🤔
                </span>
                Question: Why does a mismatch between the tool configuration and the physical tool
                lead to collisions?
              </div>
              <div className="space-y-3 text-[14px]">
                {[
                  { value: 'A', label: 'A. The target position has changed' },
                  { value: 'B', label: 'B. The program execution has changed' },
                  {
                    value: 'C',
                    label: 'C. The reference point used to align with the target has changed',
                  },
                ].map((option) => (
                  <PixelRadio
                    key={option.value}
                    label={option.label}
                    checked={selectedOption === option.value}
                    onChange={() => setSelectedOption(option.value)}
                  />
                ))}
              </div>
            </div>

            <div className="mt-8">
              <PixelButton
                variant="magenta"
                className="w-full py-4 text-[12px]"
                onClick={() => {
                  if (selectedOption !== 'C') return
                  setShowHintModal(false)
                  setSelectedOption(null)
                }}
              >
                Confirm
              </PixelButton>
            </div>
          </div>
        </div>
      )}
    </PageLayout>
  )
}

