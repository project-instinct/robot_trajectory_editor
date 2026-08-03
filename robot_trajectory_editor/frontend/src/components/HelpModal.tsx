interface HelpModalProps {
  onClose: () => void
}

export function HelpModal({ onClose }: HelpModalProps) {
  return (
    <div style={overlayStyle} onClick={onClose}>
      <div style={modalStyle} onClick={e => e.stopPropagation()}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
          <h3 style={{ margin: 0 }}>How to Use the Editor</h3>
          <button onClick={onClose}>Close</button>
        </div>
        <div style={bodyStyle}>
          <Section title="Files">
            <Item>Load Trajectory: open a <code>.npz</code> trajectory file.</Item>
            <Item>Save Trajectory: open a file-location dialog and save the current trajectory as <code>.npz</code>. You can also press Ctrl+S / Cmd+S.</Item>
            <Item>Load Terrain / Save Terrain: load (<code>.obj</code>/<code>.stl</code>) or export the terrain mesh.</Item>
            <Item>Edit Terrain: open a pop-up to crop, down-sample, move, and rotate the terrain.</Item>
            <Item>Load Robot URDF: upload a robot package when no URDF is provided at launch.</Item>
          </Section>
          <Section title="Edit Target (left panel)">
            <Item>Pick a joint, base position, or base orientation channel to edit. Its curve appears in the timeline.</Item>
          </Section>
          <Section title="Timeline (bottom)">
            <Item>Click or drag on the timeline to scrub the playhead.</Item>
            <Item>Shift + drag to select a segment; it is highlighted and targeted by Fill / Smooth / Interpolate / arrow nudges.</Item>
            <Item>Double-click to add a keyframe anchor; drag it horizontally to retime while the playhead and 3D scene follow, or vertically to change the value of the selected target.</Item>
            <Item>Right-click a keyframe to remove it.</Item>
            <Item>Mouse wheel steps the playhead.</Item>
          </Section>
          <Section title="Keyboard">
            <Item>Up / Down: nudge the selected target by the step size. If a timeline segment is selected, every frame in the segment is nudged; otherwise only the current frame.</Item>
            <Item>Left / Right: move the playhead by the frame step (no target selection needed).</Item>
            <Item>Space: toggle play / pause.</Item>
            <Item>I / O: set the selected segment's start / end to the current playhead frame. With no existing segment, I selects through the final frame and O selects from frame 0. If a new start is after the existing end, the end resets to the final frame.</Item>
            <Item>U: clear the selected segment.</Item>
            <Item>Ctrl+Z / Cmd+Z: undo the last trajectory edit.</Item>
            <Item>Ctrl+S / Cmd+S: open the dialog to save the trajectory as <code>.npz</code>.</Item>
            <Item>Esc: unfix all pinned links, or clear the selected segment.</Item>
          </Section>
          <Section title="3D Viewport">
            <Item>Drag a body link to move it; joint positions are solved by IK and written to the current frame.</Item>
            <Item>Drag the robot base to translate it; Shift + drag the base to rotate (yaw).</Item>
            <Item>Double-click links to fix their positions and orientations in the world frame. Pinned meshes are highlighted; double-click one again to unfix it, or press Esc to unfix all.</Item>
          </Section>
          <Section title="Operations (left panel)">
            <Item>Fill: fill the selected channel with its current-frame value over the selected segment (or the whole trajectory).</Item>
            <Item>Smooth: low-pass filter the selected channel over the selected segment (or the whole trajectory).</Item>
            <Item>Interpolate: linearly interpolate the selected channel between the selected segment endpoints.</Item>
            <Item>Play / Pause: play the trajectory at its framerate.</Item>
          </Section>
        </div>
      </div>
    </div>
  )
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div style={{ marginBottom: '12px' }}>
      <h4 style={{ margin: '0 0 4px 0', color: '#8ac' }}>{title}</h4>
      {children}
    </div>
  )
}

function Item({ children }: { children: React.ReactNode }) {
  return <div style={{ marginBottom: '3px', fontSize: '12px' }}>{children}</div>
}

const overlayStyle: React.CSSProperties = {
  position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
  background: 'rgba(0,0,0,0.7)', display: 'flex', alignItems: 'center', justifyContent: 'center', zIndex: 200,
}
const modalStyle: React.CSSProperties = {
  background: '#1e1e2e', border: '1px solid #444', borderRadius: '8px',
  width: '70vw', maxWidth: '640px', maxHeight: '85vh', overflow: 'auto', padding: '16px',
}
const bodyStyle: React.CSSProperties = {
  color: '#ccc', lineHeight: '1.5',
}
