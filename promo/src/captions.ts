export type Cap = {
  from: number; // seconds
  to: number;
  title: string;
  sub?: string;
  accent?: string;
  big?: boolean; // centered hero caption (intro/outro)
};

// Neon accents matching the sim's palette.
const CYAN = '#1ae6ff';
const ORANGE = '#ff8c1a';
const MAGENTA = '#ff33d9';
const GOLD = '#ffcf33';
const GREEN = '#8fe388';
const RED = '#ff3b3b';

// A caption track across the full ~1:34 recording. Event-specific lines are
// phrased generally so they stay true regardless of the exact on-screen frame;
// the Nibble line is anchored to the moment the user flagged (~0:18).
export const CAPTIONS: Cap[] = [
  {from: 0.4, to: 5.2, big: true, accent: GOLD,
    title: 'PacFleet',
    sub: 'A robot swarm that plays coordinated Pac-Man — built on real ROS 2'},
  {from: 6, to: 11.5, accent: ORANGE,
    title: 'Three robots hunt coins through a neon maze',
    sub: 'Chomp  ·  Dash  ·  Nibble'},
  {from: 12, to: 17.4, accent: CYAN,
    title: 'Each robot only sees what is inside its sensor cone',
    sub: 'field-of-view + line-of-sight, blocked by the walls'},
  {from: 18, to: 24, accent: MAGENTA,
    title: 'Nibble is low on battery → heading to its charging dock',
    sub: 'onboard FSM: patrol → chase → return-to-base'},
  {from: 25, to: 31, accent: CYAN,
    title: 'A Kalman filter fuses the noisy detections',
    sub: 'the rings show each coin’s position uncertainty'},
  {from: 32, to: 38, accent: GREEN,
    title: 'Two robots on one coin tighten the lock-on',
    sub: 'multi-sensor fusion + Hungarian data association'},
  {from: 39, to: 45, accent: GOLD,
    title: 'A market auction assigns every robot a target',
    sub: 'bids fold in distance, battery & tasking — no two overlap'},
  {from: 46, to: 52, accent: ORANGE,
    title: 'A* path planning routes them around the buildings',
    sub: 'plans on an inflated occupancy grid, replans as coins move'},
  {from: 53, to: 60, accent: GREEN,
    title: 'Capture a coin by filling its bar',
    sub: 'fills faster with more robots on it and higher certainty'},
  {from: 61, to: 68, accent: RED,
    title: 'A neural net spots the power pellet',
    sub: 'it sprints and weaves — the fleet swarms it for bonus points'},
  {from: 69, to: 76, accent: CYAN,
    title: 'Lose comms with a robot? Flagged in 3 seconds',
    sub: 'a watchdog re-auctions its coin to the rest of the fleet'},
  {from: 77, to: 84, accent: GOLD,
    title: 'One operator. An autonomous fleet.',
    sub: 'live telemetry in a custom C++ RViz panel — click a robot to focus it'},
  {from: 85, to: 94, big: true, accent: GOLD,
    title: 'ROS 2 · Kalman MOT · A* · Auctions · Neural Net · C++ RViz Panel',
    sub: 'PacFleet — built by Elad Salama'},
];
