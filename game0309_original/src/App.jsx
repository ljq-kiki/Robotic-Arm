import React from 'react'
import { PageLayout } from './components/PageLayout.jsx'
import { PixelCard } from './components/PixelCard.jsx'
import { PixelButton } from './components/PixelButton.jsx'
import { StepBar } from './components/StepBar.jsx'
import { StatDisplay } from './components/StatDisplay.jsx'

const steps = [
  {
    id: 'step-1',
    status: 'done',
    position: 'first',
    label: 'DONE',
    title: '01 INSTALL',
  },
  {
    id: 'step-2',
    status: 'active',
    position: 'middle',
    label: 'CURRENT',
    title: '02 TEST',
  },
  {
    id: 'step-3',
    status: 'pending_3',
    position: 'middle',
    label: 'WAITING',
    title: '03 MODEL',
  },
  {
    id: 'step-4',
    status: 'pending_4',
    position: 'last',
    label: 'WAITING',
    title: '04 EXECUTE',
  },
]

export default function App() {
  return (
    <PageLayout>
      <div className="flex items-center justify-between shrink-0 mb-10">
          <div className="px text-[24px] text-white/90">
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
            <div className="flex-1 min-h-0 overflow-y-auto">
              <div className="grid grid-cols-3 gap-6 shrink-0">
              <StatDisplay label="X" value="200" />
              <StatDisplay label="Y" value="100" />
              <StatDisplay label="Z" value="200" />
            </div>

            <div className="flex-1" />

            <div className="shrink-0 mt-8">
              <PixelButton
                variant="magenta"
                className="w-full py-5 px text-[12px]"
              >
                TEST TOOL
              </PixelButton>
              <div className="mt-6 text-sm" style={{ color: 'var(--muted)' }}>
                Tip: if motion drifts, try{' '}
                <span
                  className="px text-[10px]"
                  style={{ color: 'var(--magenta)' }}
                >
                  RELATIVE
                </span>{' '}
                frame.
              </div>
            </div>
            </div>
          </PixelCard>

          <PixelCard
            title="3D ROBOT MODEL"
            titleColor="var(--orange)"
            className="min-h-0 max-h-full flex flex-col overflow-hidden"
          >
            <div className="flex-1 min-h-0 overflow-y-auto">
            <div
              className="pixel-card min-h-[240px] flex items-center justify-center overflow-hidden"
              style={{
                background:
                  'linear-gradient(180deg, rgba(255,154,31,.95), rgba(255,154,31,.80))',
              }}
            >
              <div className="px text-[12px] text-black">VOXEL STAGE</div>
            </div>

            <div className="mt-6 grid grid-cols-3 gap-5 shrink-0">
              <PixelButton variant="orange" className="py-4 px text-[11px]">
                SCAN
              </PixelButton>
              <PixelButton variant="black" className="py-4 px text-[11px]">
                SNAP
              </PixelButton>
              <PixelButton variant="red" className="py-4 px text-[11px]">
                E-STOP
              </PixelButton>
            </div>
            </div>
          </PixelCard>
        </div>
    </PageLayout>
  )
}

