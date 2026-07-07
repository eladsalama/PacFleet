import React from 'react';
import {Composition} from 'remotion';
import {Pacfleet} from './Pacfleet';

// Source recording is 1502x926 @ 30fps. Shave a few px off the left edge, and
// size the composition to the exact footage length (set after probing).
export const FPS = 30;
export const SRC_W = 1502;
export const SRC_H = 926;
export const CROP_LEFT = 12; // shave the thin window-border sliver off the left
export const DURATION_FRAMES = 2824; // footage is 94.14 s @ 30 fps

export const RemotionRoot: React.FC = () => {
  return (
    <Composition
      id="Pacfleet"
      component={Pacfleet}
      durationInFrames={DURATION_FRAMES}
      fps={FPS}
      width={SRC_W - CROP_LEFT}
      height={SRC_H}
      defaultProps={{cropLeft: CROP_LEFT}}
    />
  );
};
