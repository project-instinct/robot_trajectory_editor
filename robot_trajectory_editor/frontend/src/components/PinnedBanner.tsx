import { useStore } from '../state/store'

export function PinnedBanner() {
  const pinnedLinks = useStore(s => s.pinnedLinks)
  if (pinnedLinks.length === 0) return null
  return (
    <div style={bannerStyle}>
      {pinnedLinks.length === 1 ? 'Link' : 'Links'} <strong>{pinnedLinks.join(', ')}</strong>{' '}
      {pinnedLinks.length === 1 ? 'is' : 'are'} fixed in world coordinates. Double-click a link again to unfix it, or press Esc to unfix all.
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
