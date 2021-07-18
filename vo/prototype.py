import time
import collections
import enum
from datetime import datetime
import random
import math
import pprint


class ProjectionCase(enum.Enum):
  CIRCLE = 1
  LEFT = 2
  RIGHT = 3
  COLLISION = 4


class V2:
  def __init__(self, x, y):
    self._x = x
    self._y = y

  def add(self, other):
    return V2(self._x + other._x, self._y + other._y)

  def sub(self, other):
    return V2(self._x - other._x, self._y - other._y)

  def scale(self, n):
    return V2(self._x * n, self._y * n)

  def dot(self, other):
    return self._x * other._x + self._y * other._y

  def mag2(self):
    return self.dot(self)

  def det(self, other):
    return self._x * other._y - self._y * other._x

  def __repr__(self):
    return f'<{self._x}, {self._y}>'


class CheckVO:
  def __init__(self, ap, bp, ra, rb):
    self._p = bp.sub(ap)
    self._r = ra + rb

  def r(self):
    return self._r

  def p(self) -> 'V2':
    return self._p

  def w(self, vx, vy) -> 'V2':
    return vx.sub(vy).sub(self.p())

  def __repr__(self):
    return pprint.pformat({
        'r': self.r(),
        'p': str(self.p()),
    })

  def check(self, vx, vy) -> 'ProjectionCase':
    if self.p().mag2() <= self.r() ** 2:
      return ProjectionCase.COLLISION

    w = self.w(vx, vy)

    wp = w.dot(self.p())
    if wp < 0 and wp ** 2 > w.mag2() * self.r() ** 2:
      return ProjectionCase.CIRCLE

    if self.p().det(w) > 0:
      return ProjectionCase.LEFT

    return ProjectionCase.RIGHT


class VO:
  def __init__(self, ap, bp, ra, rb):
    self._p = bp.sub(ap)
    self._r = ra + rb

    self._l2 = abs(self.p().mag2() - self.r() ** 2)

    if self.r() ** 2 > self.p().mag2():
      return

    # +- beta
    self._beta = math.acos(
        self.r() / math.sqrt(self.p().mag2()))

  def beta(self):
    return self._beta

  def l2(self):
    return self._l2

  def l(self):
    return math.sqrt(self.l2())

  def r(self):
    return self._r

  def p(self) -> 'V2':
    return self._p

  def w(self, va, vb) -> 'V2':
    # VO of A induced by B -- vb is constant (--> va - vb)
    # comparing VO -- want direction from center of VO circle to delta v --> (va - vb) - p
    return va.sub(vb).sub(self.p())

  def __repr__(self):
    return pprint.pformat({
        'r': self.r(),
        'p': str(self.p()),
    })

  def theta(self, vx, vy):
    w = self.w(vx, vy)
    wp = w.dot(self.p().scale(-1))
    return math.acos(
        wp / math.sqrt(w.mag2() * self.p().mag2()))

  def check(self, vx, vy) -> 'ProjectionCase':

    if self.p().mag2() <= self.r() ** 2:
      return ProjectionCase.COLLISION

    w = self.w(vx, vy)
    wp = w.dot(self.p().scale(-1))

    # in opposite directions of each other and less than tangent line angle
    if wp > 0 and self.theta(vx, vy) < self.beta():
      return ProjectionCase.CIRCLE

    # match CheckVO behavior, but doesn't have to (not |p x w| > 0)
    if self.p().mag2() == 0 or w.mag2() == 0:
      return ProjectionCase.RIGHT

    sintheta = w.det(self.p().scale(-1)) / math.sqrt(w.mag2() * self.p().mag2())
    if sintheta > 0:
      return ProjectionCase.LEFT

    return ProjectionCase.RIGHT  # handles if theta == 0


def _random():
  return random.random() * 200 - 100


def _test(ap, bp, ra, rb, va, vb):
  vo = VO(ap, bp, ra, rb)
  voc = CheckVO(ap, bp, ra, rb)

  vo_check = vo.check(va, vb)
  voc_check = voc.check(va, vb)

  check = {
      'VOC': voc,
      f'VOC.w({va}, {vb}': voc.w(va, vb),
      f'VOC.check({va}, {vb})': voc.check(va, vb),

      'VO': vo,
      f'VO.l()': vo.l(),
      f'VO.beta()': f'{vo.beta()} ({vo.beta() * 180 / math.pi}°)',

      f'VO.w({va}, {vb})': vo.w(va, vb),
      f'VO.wp': vo.w(va, vb).dot(vo.p().scale(-1)),
      f'VO.theta({va}, {vb})': f'{vo.theta(va, vb)} ({vo.theta(va, vb) * 180 / math.pi}°)',
      f'VO.check({va}, {vb})': vo.check(va, vb),
  }

  if vo_check != voc_check:
    raise AssertionError(
        f'{voc_check} != {vo_check}\n' + pprint.pformat(check, indent=4, sort_dicts=False)
    )

if __name__ == '__main__':
  n = int(time.time())
  print(f'seed == {n}')
  random.seed(n)

  _test(
      ap=V2(0, 0),
      bp=V2(0, 5),
      ra=1,
      rb=2,
      va=V2(0, 0),
      vb=V2(1, -1),
  )

  for _ in range(1000):
    _test(
      ap=V2(_random(), _random()),
      bp=V2(_random(), _random()),
      ra=abs(_random() + 1),
      rb=abs(_random() + 1),
      va=V2(_random(), _random()),
      vb=V2(_random(), _random()),
    )
