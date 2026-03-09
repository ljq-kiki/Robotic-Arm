import React from 'react'

const VARIANT_CLASS = {
  purple: 'btn-purple',
  orange: 'btn-orange',
  magenta: 'btn-magenta',
  black: 'btn-black',
  red: 'btn-red',
  white: 'btn-white',
}

export function PixelButton({
  variant = 'purple',
  className = '',
  children,
  ...rest
}) {
  const colorClass = VARIANT_CLASS[variant] ?? VARIANT_CLASS.purple

  return (
    <button
      className={`pixel-btn px ${colorClass} ${className}`}
      type="button"
      {...rest}
    >
      {children}
    </button>
  )
}

