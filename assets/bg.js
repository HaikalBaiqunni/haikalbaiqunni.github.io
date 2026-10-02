/* Physical-AI background: neural node graph + 3-DOF robot arm (FABRIK IK).
   - Cursor active  -> arm tracks the pointer
   - Idle           -> arm runs an autonomous pick-and-place loop
   Respects prefers-reduced-motion (renders one static frame). */
(() => {
  'use strict';
  const canvas = document.getElementById('bg');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;

  const INK = '231,238,243';
  const ACC = '216,154,90';
  const BG = '#12293F';
  const LINK = 130;
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  const rnd = (a, b) => a + Math.random() * (b - a);
  const pad = (n, w) => String(Math.round(n)).padStart(w, ' ');

  let W = 0, H = 0, dpr = 1, small = false, scrollY = 0;
  let nodes = [], pulses = [];
  const ptr = { x: -1e4, y: -1e4, last: -1e9 };

  /* ---------- robot arm ---------- */
  const arm = {
    base: { x: 0, y: 0 }, len: [0, 0, 0], reach: 0,
    p: [], tg: { x: 0, y: 0 }, grip: 0,
    phase: 'toA', timer: 0, A: { x: 0, y: 0 }, B: { x: 0, y: 0 },
    obj: { x: 0, y: 0 }, held: false, mode: 'AUTO', err: 0
  };

  function pickWorkspacePoint() {
    const a = rnd(Math.PI + 0.12, Math.PI * 1.5 - 0.06);
    const r = rnd(0.45, 0.88) * arm.reach;
    return {
      x: Math.max(24, arm.base.x + Math.cos(a) * r),
      y: arm.base.y + Math.sin(a) * r
    };
  }

  function newCycle() {
    arm.A = pickWorkspacePoint();
    do { arm.B = pickWorkspacePoint(); }
    while (Math.hypot(arm.A.x - arm.B.x, arm.A.y - arm.B.y) < arm.reach * 0.3);
    arm.obj = { x: arm.A.x, y: arm.A.y };
    arm.held = false;
    arm.phase = 'toA';
    arm.timer = 0;
  }

  function initArm() {
    const L = clamp(Math.min(W, H) * 0.17, small ? 46 : 70, 150);
    arm.len = [L * 1.1, L, L * 0.7];
    arm.reach = arm.len[0] + arm.len[1] + arm.len[2];
    arm.base = { x: W - (small ? 44 : 90), y: H - (small ? 24 : 28) };
    const b = arm.base;
    arm.p = [
      { x: b.x, y: b.y },
      { x: b.x - arm.len[0] * 0.3, y: b.y - arm.len[0] * 0.95 },
      { x: b.x - arm.len[0] * 0.6 - arm.len[1] * 0.5, y: b.y - arm.len[0] * 0.9 - arm.len[1] * 0.5 },
      { x: b.x - arm.len[0] * 0.7 - arm.len[1] * 0.9, y: b.y - arm.len[0] * 0.8 - arm.len[1] * 0.6 }
    ];
    arm.tg = { x: arm.p[3].x, y: arm.p[3].y };
    newCycle();
  }

  function fabrik(t) {
    const p = arm.p, L = arm.len, b = arm.base;
    const dx = t.x - b.x, dy = t.y - b.y;
    const d = Math.hypot(dx, dy);
    if (d >= arm.reach) {
      const k = arm.reach / d;
      t = { x: b.x + dx * k, y: b.y + dy * k };
    }
    for (let it = 0; it < 8; it++) {
      p[3].x = t.x; p[3].y = t.y;
      for (let i = 2; i >= 0; i--) {
        const vx = p[i].x - p[i + 1].x, vy = p[i].y - p[i + 1].y;
        const m = Math.hypot(vx, vy) || 1;
        p[i].x = p[i + 1].x + vx / m * L[i];
        p[i].y = p[i + 1].y + vy / m * L[i];
      }
      p[0].x = b.x; p[0].y = b.y;
      for (let i = 0; i < 3; i++) {
        const vx = p[i + 1].x - p[i].x, vy = p[i + 1].y - p[i].y;
        const m = Math.hypot(vx, vy) || 1;
        p[i + 1].x = p[i].x + vx / m * L[i];
        p[i + 1].y = p[i].y + vy / m * L[i];
      }
    }
  }

  function stepArm(dt, now) {
    const tracking = now - ptr.last < 2500;
    const wasTracking = arm.mode === 'TRACKING';
    arm.mode = tracking ? 'TRACKING' : 'AUTONOMOUS';
    if (!tracking && wasTracking) newCycle();

    const end = arm.p[3];
    let goal, gripTarget = 0;

    if (tracking) {
      goal = { x: ptr.x, y: ptr.y };
      arm.held = false;
    } else {
      switch (arm.phase) {
        case 'toA':
          goal = arm.A;
          if (Math.hypot(end.x - arm.A.x, end.y - arm.A.y) < 10) { arm.phase = 'grasp'; arm.timer = 0; }
          break;
        case 'grasp':
          goal = arm.A; arm.timer += dt;
          if (arm.timer > 0.2) gripTarget = 1;
          if (arm.timer > 0.55) { arm.phase = 'toB'; arm.held = true; }
          break;
        case 'toB':
          goal = arm.B; gripTarget = 1;
          if (Math.hypot(end.x - arm.B.x, end.y - arm.B.y) < 10) {
            arm.phase = 'release'; arm.timer = 0; arm.held = false;
            arm.obj = { x: arm.B.x, y: arm.B.y };
          }
          break;
        case 'release':
          goal = arm.B; arm.timer += dt;
          if (arm.timer > 0.5) { arm.phase = 'rest'; arm.timer = 0; }
          break;
        default: // rest
          goal = { x: arm.base.x - arm.reach * 0.5, y: arm.base.y - arm.reach * 0.55 };
          arm.timer += dt;
          if (arm.timer > 1.3) newCycle();
      }
      if (arm.held) { arm.obj.x = end.x; arm.obj.y = end.y; }
    }

    const k = 1 - Math.exp(-dt * (tracking ? 7 : 4.2));
    arm.tg.x += (goal.x - arm.tg.x) * k;
    arm.tg.y += (goal.y - arm.tg.y) * k;
    fabrik(arm.tg);
    arm.grip += (gripTarget - arm.grip) * (1 - Math.exp(-dt * 12));
    arm.err = Math.hypot(goal.x - end.x, goal.y - end.y);
    arm.goal = goal;
  }

  /* ---------- node graph ---------- */
  function initNodes() {
    const n = clamp(Math.round(W * H / (small ? 26000 : 17000)), 22, 90);
    nodes = Array.from({ length: n }, () => ({
      x: rnd(0, W), y: rnd(0, H),
      vx: rnd(-12, 12), vy: rnd(-12, 12), r: rnd(1, 2.1)
    }));
    pulses = [];
  }

  function stepNodes(dt) {
    for (const n of nodes) {
      n.x += n.vx * dt; n.y += n.vy * dt;
      if (n.x < -20) n.x = W + 20; else if (n.x > W + 20) n.x = -20;
      if (n.y < -20) n.y = H + 20; else if (n.y > H + 20) n.y = -20;
    }
  }

  /* ---------- drawing ---------- */
  function drawNodes(dt, active) {
    const off = scrollY * 0.12;
    const pos = nodes.map(n => ({ x: n.x, y: (((n.y - off) % H) + H) % H }));

    ctx.lineWidth = 1;
    for (let i = 0; i < pos.length; i++) {
      for (let j = i + 1; j < pos.length; j++) {
        const dx = pos[i].x - pos[j].x, dy = pos[i].y - pos[j].y;
        const d2 = dx * dx + dy * dy;
        if (d2 < LINK * LINK) {
          ctx.strokeStyle = `rgba(${INK},${(1 - Math.sqrt(d2) / LINK) * 0.16})`;
          ctx.beginPath(); ctx.moveTo(pos[i].x, pos[i].y); ctx.lineTo(pos[j].x, pos[j].y); ctx.stroke();
        }
      }
    }

    if (active) {
      for (const p of pos) {
        const d = Math.hypot(p.x - ptr.x, p.y - ptr.y);
        if (d < 170) {
          ctx.strokeStyle = `rgba(${ACC},${(1 - d / 170) * 0.4})`;
          ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(ptr.x, ptr.y); ctx.stroke();
        }
      }
    }

    for (let i = 0; i < pos.length; i++) {
      ctx.fillStyle = `rgba(${INK},0.4)`;
      ctx.beginPath(); ctx.arc(pos[i].x, pos[i].y, nodes[i].r, 0, 6.2832); ctx.fill();
    }

    // signal pulses travelling along links
    if (!reduce && pulses.length < 8 && Math.random() < 0.04) {
      const a = (Math.random() * pos.length) | 0;
      for (let j = 0; j < pos.length; j++) {
        if (j !== a && Math.hypot(pos[a].x - pos[j].x, pos[a].y - pos[j].y) < LINK) {
          pulses.push({ a, b: j, t: 0 }); break;
        }
      }
    }
    for (let i = pulses.length - 1; i >= 0; i--) {
      const q = pulses[i];
      q.t += dt * 1.3;
      if (q.t >= 1) { pulses.splice(i, 1); continue; }
      const A = pos[q.a], B = pos[q.b];
      const x = A.x + (B.x - A.x) * q.t, y = A.y + (B.y - A.y) * q.t;
      const g = ctx.createRadialGradient(x, y, 0, x, y, 9);
      g.addColorStop(0, `rgba(${ACC},0.9)`); g.addColorStop(1, `rgba(${ACC},0)`);
      ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, 9, 0, 6.2832); ctx.fill();
    }
  }

  function dashedLine(x1, y1, x2, y2, alpha) {
    ctx.setLineDash([4, 5]);
    ctx.strokeStyle = `rgba(${ACC},${alpha})`;
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.setLineDash([]);
  }

  function drawReticle(x, y, label, t) {
    ctx.strokeStyle = `rgba(${ACC},0.75)`; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 6.2832); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x - 15, y); ctx.lineTo(x - 5, y); ctx.moveTo(x + 5, y); ctx.lineTo(x + 15, y);
    ctx.moveTo(x, y - 15); ctx.lineTo(x, y - 5); ctx.moveTo(x, y + 5); ctx.lineTo(x, y + 15);
    ctx.stroke();
    const ph = (t * 0.9) % 1;
    ctx.strokeStyle = `rgba(${ACC},${(1 - ph) * 0.5})`;
    ctx.beginPath(); ctx.arc(x, y, 9 + ph * 30, 0, 6.2832); ctx.stroke();
    if (!small) {
      ctx.fillStyle = `rgba(${ACC},0.8)`;
      ctx.fillText(label, x + 18, y - 14);
    }
  }

  function drawArm(t) {
    const p = arm.p, b = arm.base;

    // planned path + goal
    if (arm.err > 14 && arm.goal) dashedLine(p[3].x, p[3].y, arm.goal.x, arm.goal.y, 0.35);
    if (arm.mode === 'AUTONOMOUS' && arm.phase === 'toB') {
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = `rgba(${ACC},0.4)`;
      ctx.strokeRect(arm.B.x - 9, arm.B.y - 9, 18, 18);
      ctx.setLineDash([]);
    }

    // base
    ctx.fillStyle = BG; ctx.strokeStyle = `rgba(${INK},0.4)`; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.rect(b.x - 26, b.y + 2, 52, 14); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.arc(b.x, b.y + 2, 14, Math.PI, 0); ctx.fill(); ctx.stroke();

    // links (outlined tube look)
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    const widths = [13, 11, 9];
    for (let i = 0; i < 3; i++) {
      ctx.strokeStyle = `rgba(${INK},0.4)`; ctx.lineWidth = widths[i];
      ctx.beginPath(); ctx.moveTo(p[i].x, p[i].y); ctx.lineTo(p[i + 1].x, p[i + 1].y); ctx.stroke();
      ctx.strokeStyle = BG; ctx.lineWidth = widths[i] - 3;
      ctx.beginPath(); ctx.moveTo(p[i].x, p[i].y); ctx.lineTo(p[i + 1].x, p[i + 1].y); ctx.stroke();
    }
    // joints
    for (let i = 0; i < 3; i++) {
      ctx.fillStyle = BG; ctx.strokeStyle = `rgba(${ACC},0.85)`; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(p[i].x, p[i].y, 7, 0, 6.2832); ctx.fill(); ctx.stroke();
      ctx.fillStyle = `rgba(${ACC},0.9)`;
      ctx.beginPath(); ctx.arc(p[i].x, p[i].y, 2, 0, 6.2832); ctx.fill();
    }

    // gripper
    const ang = Math.atan2(p[3].y - p[2].y, p[3].x - p[2].x);
    const fx = Math.cos(ang), fy = Math.sin(ang), nx = -fy, ny = fx;
    const w = 10 - arm.grip * 7;
    ctx.strokeStyle = `rgba(${ACC},0.9)`; ctx.lineWidth = 2.5;
    for (const s of [-1, 1]) {
      ctx.beginPath();
      ctx.moveTo(p[3].x + nx * w * s, p[3].y + ny * w * s);
      ctx.lineTo(p[3].x + nx * w * s + fx * 11, p[3].y + ny * w * s + fy * 11);
      ctx.moveTo(p[3].x - nx * 0 + nx * w * s, p[3].y + ny * w * s);
      ctx.lineTo(p[3].x, p[3].y);
      ctx.stroke();
    }
    ctx.lineCap = 'butt';

    // object being manipulated
    if (arm.mode === 'AUTONOMOUS') {
      const o = arm.obj, s = arm.held ? 7 : 8;
      ctx.fillStyle = `rgba(${ACC},0.18)`; ctx.strokeStyle = `rgba(${ACC},0.95)`; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.rect(o.x - s, o.y - s, s * 2, s * 2); ctx.fill(); ctx.stroke();
    }

    // target reticle
    if (arm.goal) {
      const lbl = arm.mode === 'TRACKING' ? 'CURSOR'
        : arm.phase === 'toA' || arm.phase === 'grasp' ? 'PICK' : arm.phase === 'rest' ? 'HOME' : 'PLACE';
      drawReticle(arm.goal.x, arm.goal.y, lbl, t);
    }
  }

  function wrapDeg(d) { d = ((d + 180) % 360 + 360) % 360 - 180; return d; }

  function drawHUD() {
    if (small) return;
    const p = arm.p;
    const a = [0, 1, 2].map(i => Math.atan2(p[i + 1].y - p[i].y, p[i + 1].x - p[i].x) * 180 / Math.PI);
    const q = [wrapDeg(a[0] + 90), wrapDeg(a[1] - a[0]), wrapDeg(a[2] - a[1])];
    const lines = [
      'PHYSICAL-AI // ARM-3DOF SIM',
      'MODE  ' + arm.mode,
      'q     ' + q.map(v => pad(v, 4)).join(' ') + ' deg',
      'ERR   ' + arm.err.toFixed(1) + ' px'
    ];
    ctx.fillStyle = `rgba(${INK},0.4)`;
    lines.forEach((s, i) => ctx.fillText(s, 18, H - 66 + i * 14));
  }

  function drawScan(t) {
    const y = ((t * 55) % (H + 240)) - 120;
    const g = ctx.createLinearGradient(0, y - 60, 0, y);
    g.addColorStop(0, `rgba(${ACC},0)`); g.addColorStop(1, `rgba(${ACC},0.045)`);
    ctx.fillStyle = g; ctx.fillRect(0, y - 60, W, 60);
    ctx.fillStyle = `rgba(${ACC},0.1)`; ctx.fillRect(0, y, W, 1);
  }

  function drawGlow(active) {
    if (!active) return;
    const g = ctx.createRadialGradient(ptr.x, ptr.y, 0, ptr.x, ptr.y, 220);
    g.addColorStop(0, `rgba(${ACC},0.07)`); g.addColorStop(1, `rgba(${ACC},0)`);
    ctx.fillStyle = g; ctx.fillRect(ptr.x - 220, ptr.y - 220, 440, 440);
  }

  /* ---------- main loop ---------- */
  let last = 0, clock = 0;

  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000 || 0.016);
    last = now; clock += dt;
    render(dt, now);
    if (!reduce) requestAnimationFrame(frame);
  }

  function render(dt, now) {
    const active = now - ptr.last < 2500;
    stepNodes(dt);
    stepArm(dt, now);
    ctx.clearRect(0, 0, W, H);
    ctx.font = '10px "JetBrains Mono", ui-monospace, monospace';
    drawGlow(active);
    drawScan(clock);
    drawNodes(dt, active);
    drawArm(clock);
    drawHUD();
  }

  function resize() {
    const wChanged = innerWidth !== W;
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    W = innerWidth; H = innerHeight; small = W < 640;
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (wChanged || !nodes.length) { initNodes(); initArm(); }
    else { arm.base.y = H - (small ? 24 : 28); }
    if (reduce) settleStatic();
  }

  function settleStatic() {
    for (let i = 0; i < 240; i++) { stepNodes(0.016); stepArm(0.016, 0); }
    render(0.016, 0);
  }

  addEventListener('resize', resize);
  addEventListener('scroll', () => { scrollY = window.scrollY; }, { passive: true });
  addEventListener('pointermove', e => { ptr.x = e.clientX; ptr.y = e.clientY; ptr.last = performance.now(); }, { passive: true });
  document.addEventListener('visibilitychange', () => { last = performance.now(); });

  resize();
  if (!reduce) requestAnimationFrame(t => { last = t; frame(t); });
})();
