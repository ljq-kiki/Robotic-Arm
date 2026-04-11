import React from 'react'

export function PixelCard({
  title,
  titleColor,
  titleBadgeText,
  children,
  className = '',
  padding = 'p-8',
}) {
  const colors = Array.isArray(titleColor) ? titleColor : titleColor ? [titleColor] : []
  const hasHeader = title || colors.length > 0 || titleBadgeText
  const badgeColor = colors[0] ?? 'var(--orange)'

  return (
    <section className={`pixel-card soft-grid ${padding} flex flex-col ${className}`}>
      {hasHeader && (
        <div className="flex items-center justify-between mb-6 shrink-0">
          {title || titleBadgeText ? (
            <div className="flex items-center gap-3">
              {titleBadgeText && (
                <div
                  className="swatch swatch-badge px flex items-center justify-center text-[12px] font-bold leading-none"
                  style={{ background: badgeColor }}
                  aria-label={`Badge ${titleBadgeText}`}
                >
                  {titleBadgeText}
                </div>
              )}
              {title ? <div className="px text-[12px]">{title}</div> : null}
            </div>
          ) : (
            <div />
          )}
          {!titleBadgeText && colors.length > 0 && (
            <div className="flex gap-2">
              {colors.map((color, index) => (
                <div
                  key={`${color}-${index}`}
                  className="swatch"
                  style={{ background: color }}
                />
              ))}
            </div>
          )}
        </div>
      )}
      {children}
    </section>
  )
}

