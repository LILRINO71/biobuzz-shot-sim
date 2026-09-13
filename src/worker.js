/* Worker wrapper around ShotEngine (engine source is prepended at build time). */
(function () {
  'use strict';
  var E = self.ShotEngine;
  var scanToken = 0;

  function reply(msg) { self.postMessage(msg); }

  function runScan(id, params, points, i, token) {
    if (token !== scanToken) return;
    var end = Math.min(points.length, i + 8);
    for (; i < end; i++) {
      var q = points[i], p = Object.assign({}, params, { robot: q });
      var r = E.evaluate(p, 'coarse');
      reply({ type: 'scanCell', id: id, x: q.x, y: q.y, verdict: r.verdict, hitRate: r.hitRate, done: i + 1, total: points.length });
    }
    if (i < points.length) setTimeout(function () { runScan(id, params, points, i, token); }, 0);
    else reply({ type: 'scanDone', id: id });
  }

  self.onmessage = function (ev) {
    var m = ev.data || {};
    try {
      if (m.type === 'init') {
        E.init(m.data);
        reply({ type: 'ready' });
      } else if (m.type === 'table') {
        E.buildTable(m.ballId, m.S0);
        reply({ type: 'tableReady', id: m.id });
      } else if (m.type === 'evaluate') {
        reply({ type: 'result', id: m.id, mode: m.mode, result: E.evaluate(m.params, m.mode) });
      } else if (m.type === 'motors') {
        reply({ type: 'motors', id: m.id, rows: E.compareMotors(m.params) });
      } else if (m.type === 'scan') {
        scanToken++;
        runScan(m.id, m.params, E.scanPoints(m.step || 6), 0, scanToken);
      } else if (m.type === 'cancelScan') {
        scanToken++;
      }
    } catch (err) {
      reply({ type: 'error', id: m.id, message: String((err && err.message) || err) });
    }
  };
})();
