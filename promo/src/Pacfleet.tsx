import React from 'react';
import {
  AbsoluteFill,
  OffthreadVideo,
  Sequence,
  staticFile,
  useVideoConfig,
} from 'remotion';
import {CAPTIONS} from './captions';
import {Caption} from './Caption';
import {SRC_W, SRC_H} from './Root';

export const Pacfleet: React.FC<{cropLeft: number}> = ({cropLeft}) => {
  const {fps} = useVideoConfig();
  return (
    <AbsoluteFill style={{backgroundColor: '#05060a', overflow: 'hidden'}}>
      {/* the RViz recording, audio stripped (muted), left edge shaved off */}
      <OffthreadVideo
        src={staticFile('footage.mp4')}
        muted
        style={{
          position: 'absolute',
          left: -cropLeft,
          top: 0,
          width: SRC_W,
          height: SRC_H,
        }}
      />

      {/* a subtle vignette so bottom captions stay readable */}
      <AbsoluteFill
        style={{
          background:
            'linear-gradient(to bottom, rgba(0,0,0,0) 55%, rgba(0,0,0,0.45) 100%)',
        }}
      />

      {CAPTIONS.map((c, i) => (
        <Sequence
          key={i}
          from={Math.round(c.from * fps)}
          durationInFrames={Math.round((c.to - c.from) * fps)}
        >
          <Caption title={c.title} sub={c.sub} accent={c.accent} big={c.big} />
        </Sequence>
      ))}
    </AbsoluteFill>
  );
};
