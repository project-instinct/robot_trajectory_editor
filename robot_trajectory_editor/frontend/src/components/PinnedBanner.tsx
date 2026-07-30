import { useStore } from '../state/store'

export function PinnedBanner() {
  const pinnedLink = useStore(s => s.pinnedLink)
  if (!pinnedLink) return null
  return (
    <div style={bannerStyle}>
      Link <strong>{pinnedLink}</strong> is fixed in world coordinates. Double-click again or press Esc to unfix.
    </div>
  )
}

const bannerStyle: React.CSSProperties = {
  position: 'absolute',
  top: 0,
  left: 0,
  right: 0,
  background: '#ffaa00',
  color: '#000',
  padding: '4px 12px',
  textAlign: 'center',
  fontSize: '13px',
  fontWeight: 'bold',
  zIndex: 100,
}
