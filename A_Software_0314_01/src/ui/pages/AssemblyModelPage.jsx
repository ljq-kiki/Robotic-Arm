import React, { useState } from 'react'
import { PageLayout } from '../components/PageLayout.jsx'
import { PixelCard } from '../components/PixelCard.jsx'
import { PixelButton } from '../components/PixelButton.jsx'
import { StepBar } from '../components/StepBar.jsx'
import { PixelSelect } from '../components/PixelSelect.jsx'
import { PixelInput } from '../components/PixelInput.jsx'

const steps = [
  { id: 'step-1', status: 'done', position: 'first', label: '', title: 'INSTALL' },
  { id: 'step-2', status: 'done', position: 'middle', label: '', title: 'TEST' },
  { id: 'step-3', status: 'active', position: 'middle', label: '', title: 'ASSEMBLY' },
  { id: 'step-4', status: 'pending_4', position: 'last', label: '', title: 'EXECUTE' },
]

const EMPTY_POINT = { x: '', y: '', z: '', rx: '' }

function isPointFilled(point) {
  return point.x && point.y && point.z && point.rx
}

function mockHardwareData() {
  return {
    x: '200.00',
    y: '200.00',
    z: '200.00',
    rx: '200.00',
  }
}

export default function AssemblyModelPage() {
  const [mode, setMode] = useState('pick') // pick | drop
  const [grab, setGrab] = useState(EMPTY_POINT)
  const [drop, setDrop] = useState(EMPTY_POINT)
  const [waypoints, setWaypoints] = useState([])
  const [nextId, setNextId] = useState(1)

  const canConfirm = isPointFilled(grab) && isPointFilled(drop)

  const handleTogglePickDrop = () => {
    setMode((prev) => (prev === 'pick' ? 'drop' : 'pick'))
  }

  const handleAddWaypoint = () => {
    setWaypoints((prev) => [
      ...prev,
      { id: nextId, point: { ...EMPTY_POINT } },
    ])
    setNextId((id) => id + 1)
  }

  const handleRemoveWaypoint = (id) => {
    setWaypoints((prev) => prev.filter((w) => w.id !== id))
  }

  const handleRecord = (targetSetter) => {
    targetSetter(mockHardwareData())
  }

  const handleRecordWaypoint = (id) => {
    const data = mockHardwareData()
    setWaypoints((prev) =>
      prev.map((w) => (w.id === id ? { ...w, point: data } : w)),
    )
  }

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

      <div className="grid lg:grid-cols-2 gap-8 flex-1 min-h-0 max-lg:grid-rows-[minmax(0,1fr)_minmax(0,1fr)]">
        <PixelCard
          padding="p-6"
          className="min-h-0 max-h-full flex flex-col overflow-hidden"
        >
          <div className="flex-1 min-h-0 overflow-y-auto flex flex-col gap-4 pr-6 pb-6">
            <div className="text-[13px]" style={{ color: 'var(--magenta)' }}>
              Use the mini robot to control the robot arm to pick and assemble the first block.
            </div>

            <div
              className="pixel-card soft-grid mt-2"
              style={{
                background: 'var(--panel)',
                minHeight: '160px',
              }}
            >
              <div className="w-full h-full flex items-center justify-center">
                <div className="text-center">
                  <div className="px text-[12px] mb-2">GIF PLACEHOLDER</div>
                  <div className="text-[10px]" style={{ color: 'var(--muted)' }}>
                    How to teleoperate the mini arm
                  </div>
                </div>
              </div>
            </div>

            <PixelCard padding="p-4" className="mt-4 flex-1 min-h-0 overflow-hidden">
              <div className="text-[13px] mb-3">Trajectory Planning</div>

              <div className="flex-1 min-h-0 overflow-y-auto flex flex-col gap-3">

              {/* Grab Point */}
              <div
                className="flex gap-3 p-2 pb-3 border-b border-dashed border-[rgba(0,0,0,0.2)]"
                style={{ background: '#F5F5F5', borderRadius: '6px' }}
              >
                <div className="flex-1 flex flex-col gap-2">
                  <div className="flex items-center gap-2">
                    <span style={{ color: '#FF2DAA' }}>●</span>
                    <span className="text-[13px] font-semibold">Grab Point</span>
                    <span className="text-[12px] ml-4">Reference Frame</span>
                    <PixelSelect
                      className="w-[120px]"
                      variant="flat"
                      options={[
                        { value: 'Base', label: 'Base' },
                        { value: 'World', label: 'World' },
                      ]}
                      defaultValue="Base"
                    />
                  </div>
                  <div className="grid grid-cols-4 gap-2">
                    {['X', 'Y', 'Z', 'Rx'].map((axis) => (
                      <div key={axis} className="flex items-center gap-1 min-w-0">
                        <span className="text-[12px] font-semibold shrink-0">{axis}</span>
                        <PixelInput
                          className="min-w-0 flex-1"
                          value={grab[axis.toLowerCase()]}
                          variant="flat"
                          onChange={(val) =>
                            setGrab((prev) => ({ ...prev, [axis.toLowerCase()]: val }))
                          }
                        />
                      </div>
                    ))}
                  </div>
                </div>
                <div
                  className="pl-3 flex items-center justify-center"
                  style={{ borderLeft: '1px solid rgba(0,0,0,0.15)' }}
                >
                  <PixelButton
                    variant="outline"
                    className="text-[12px]"
                    onClick={() => handleRecord(setGrab)}
                  >
                    Record
                  </PixelButton>
                </div>
              </div>

              {/* Waypoints */}
              {waypoints.map((w) => (
                <div
                  key={w.id}
                  className="relative flex gap-3 p-2 py-3 border-b border-dashed border-[rgba(0,0,0,0.2)]"
                  style={{ background: '#F5F5F5', borderRadius: '6px' }}
                >
                  <button
                    type="button"
                    onClick={() => handleRemoveWaypoint(w.id)}
                    className="absolute top-2 right-2 w-6 h-6 rounded-full border-2 border-[var(--ink)] flex items-center justify-center text-[16px] z-10"
                    style={{ background: 'var(--panel)' }}
                    aria-label="Remove waypoint"
                  >
                    -
                  </button>
                  <div className="flex-1 flex flex-col gap-2">
                    <div className="flex items-center gap-2">
                      <span style={{ color: '#FF9A1F' }}>●</span>
                      <span className="text-[13px] font-semibold">WayPoint</span>
                      <span className="text-[12px] ml-4">Reference Frame</span>
                      <PixelSelect
                        className="w-[120px]"
                        variant="flat"
                        options={[
                          { value: 'Base', label: 'Base' },
                          { value: 'World', label: 'World' },
                        ]}
                        defaultValue="Base"
                      />
                    </div>
                    <div className="grid grid-cols-4 gap-2">
                      {['X', 'Y', 'Z', 'Rx'].map((axis) => (
                        <div key={axis} className="flex items-center gap-1 min-w-0">
                          <span className="text-[12px] font-semibold shrink-0">{axis}</span>
                          <PixelInput
                            className="min-w-0 flex-1"
                            value={w.point[axis.toLowerCase()]}
                            variant="flat"
                            onChange={(val) =>
                              setWaypoints((prev) =>
                                prev.map((item) =>
                                  item.id === w.id
                                    ? {
                                        ...item,
                                        point: {
                                          ...item.point,
                                          [axis.toLowerCase()]: val,
                                        },
                                      }
                                    : item,
                                ),
                              )
                            }
                          />
                        </div>
                      ))}
                    </div>
                  </div>
                  <div
                    className="pl-3 flex items-center justify-center"
                    style={{ borderLeft: '1px solid rgba(0,0,0,0.15)' }}
                  >
                    <PixelButton
                      variant="outline"
                      className="text-[12px]"
                      onClick={() => handleRecordWaypoint(w.id)}
                    >
                      Record
                    </PixelButton>
                  </div>
                </div>
              ))}

              {/* Add waypoint button */}
              <div className="flex justify-center my-2">
                <button
                  type="button"
                  onClick={handleAddWaypoint}
                  className="w-8 h-8 rounded-full border-2 border-[var(--ink)] flex items-center justify-center text-[18px]"
                  style={{ background: 'var(--panel)' }}
                  aria-label="Add waypoint"
                >
                  +
                </button>
              </div>

              {/* Drop Point */}
              <div
                className="flex gap-3 p-2 pt-3"
                style={{ background: '#F5F5F5', borderRadius: '6px' }}
              >
                <div className="flex-1 flex flex-col gap-2">
                  <div className="flex items-center gap-2">
                    <span style={{ color: '#00C853' }}>●</span>
                    <span className="text-[13px] font-semibold">Drop Point</span>
                    <span className="text-[12px] ml-4">Reference Frame</span>
                    <PixelSelect
                      className="w-[120px]"
                      variant="flat"
                      options={[
                        { value: 'Base', label: 'Base' },
                        { value: 'World', label: 'World' },
                      ]}
                      defaultValue="Base"
                    />
                  </div>
                  <div className="grid grid-cols-4 gap-2">
                    {['X', 'Y', 'Z', 'Rx'].map((axis) => (
                      <div key={axis} className="flex items-center gap-1 min-w-0">
                        <span className="text-[12px] font-semibold shrink-0">{axis}</span>
                        <PixelInput
                          className="min-w-0 flex-1"
                          value={drop[axis.toLowerCase()]}
                          variant="flat"
                          onChange={(val) =>
                            setDrop((prev) => ({ ...prev, [axis.toLowerCase()]: val }))
                          }
                        />
                      </div>
                    ))}
                  </div>
                </div>
                <div
                  className="pl-3 flex items-center justify-center"
                  style={{ borderLeft: '1px solid rgba(0,0,0,0.15)' }}
                >
                  <PixelButton
                    variant="outline"
                    className="text-[12px]"
                    onClick={() => handleRecord(setDrop)}
                  >
                    Record
                  </PixelButton>
                </div>
              </div>
              </div>
            </PixelCard>
          </div>

          <div className="mt-4 flex items-center gap-4">
            <PixelButton
              variant="white"
              className="px-8 py-4 text-[12px]"
              onClick={handleTogglePickDrop}
            >
              {mode === 'pick' ? 'Pick' : 'Drop'}
            </PixelButton>
            <PixelButton
              variant="magenta"
              className={`flex-1 py-4 text-[12px] ${
                !canConfirm ? 'opacity-40 pointer-events-none' : ''
              }`}
              disabled={!canConfirm}
            >
              Confirm &amp; Test
            </PixelButton>
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
                <div className="px text-[16px] mb-2">Lion Model</div>
                <div className="px text-[12px]" style={{ color: 'var(--muted)' }}>
                  Install the first building block
                </div>
              </div>
            </div>
          </div>
        </PixelCard>
      </div>
    </PageLayout>
  )
}

