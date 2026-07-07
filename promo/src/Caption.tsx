import React from 'react';
import {
  AbsoluteFill,
  interpolate,
  spring,
  useCurrentFrame,
  useVideoConfig,
} from 'remotion';

const FONT =
  '"Segoe UI", system-ui, -apple-system, Roboto, Helvetica, Arial, sans-serif';

export const Caption: React.FC<{
  title: string;
  sub?: string;
  accent?: string;
  big?: boolean;
}> = ({title, sub, accent = '#1ae6ff', big = false}) => {
  const frame = useCurrentFrame();
  const {fps, durationInFrames} = useVideoConfig();

  const enter = spring({frame, fps, config: {damping: 200}, durationInFrames: 12});
  const exit = interpolate(
    frame,
    [durationInFrames - 10, durationInFrames],
    [1, 0],
    {extrapolateLeft: 'clamp', extrapolateRight: 'clamp'},
  );
  const opacity = enter * exit;
  const slide = interpolate(enter, [0, 1], [big ? 20 : 40, 0]);

  const pill: React.CSSProperties = big
    ? {
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        gap: 18,
        padding: '34px 56px',
        borderRadius: 22,
        background: 'rgba(5, 8, 16, 0.72)',
        border: `2px solid ${accent}`,
        boxShadow: `0 0 40px ${accent}55, inset 0 0 30px rgba(0,0,0,0.4)`,
        textAlign: 'center',
        maxWidth: '82%',
      }
    : {
        display: 'flex',
        flexDirection: 'column',
        gap: 8,
        padding: '18px 26px 18px 26px',
        borderRadius: 14,
        background: 'rgba(6, 10, 18, 0.82)',
        borderLeft: `6px solid ${accent}`,
        boxShadow: '0 10px 30px rgba(0,0,0,0.45)',
        maxWidth: '78%',
      };

  return (
    <AbsoluteFill
      style={{
        justifyContent: big ? 'center' : 'flex-end',
        alignItems: 'center',
        paddingBottom: big ? 0 : 70,
        fontFamily: FONT,
      }}
    >
      <div style={{...pill, opacity, transform: `translateY(${slide}px)`}}>
        <div
          style={{
            color: '#ffffff',
            fontWeight: 800,
            fontSize: big ? 60 : 40,
            lineHeight: 1.12,
            letterSpacing: big ? 1 : 0.2,
            textShadow: `0 0 18px ${accent}66`,
          }}
        >
          {title}
        </div>
        {sub ? (
          <div
            style={{
              color: '#b9c7d6',
              fontWeight: 500,
              fontSize: big ? 28 : 24,
              lineHeight: 1.25,
            }}
          >
            {sub}
          </div>
        ) : null}
      </div>
    </AbsoluteFill>
  );
};
