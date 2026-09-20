"""Two-item inventory and floor-only random test scheduling; no motion commands."""
from copy import deepcopy
from packing_env.data_type.item import Item
from packing_env.data_type.geometry import Point3D


def disjoint_xy(corner, size, record):
    other, extent = record['virtual_corner_mm'], record['virtual_size_mm']
    return any(corner[i]+size[i] <= other[i]+1e-6 or
               other[i]+extent[i] <= corner[i]+1e-6 for i in (0, 1))


class TwoItemTest:
    def _two_enabled(self):
        return getattr(self, 'two_item_mode', False)

    def _sync_item(self):
        if self._two_enabled() and self.record is not None:
            self.test_items[self.active_item] = dict(location=self.location, record=deepcopy(self.record))

    def _select_item(self, key):
        self.active_item = key
        entry = self.test_items.get(key)
        self.location = entry['location'] if entry else 'empty'
        self.record = deepcopy(entry['record']) if entry else None
        if self.location == 'slot':
            self.test_slot = self.record['slot_id']

    def set_two_item_mode(self, request, response):
        if (self.busy or self.random_active or self.state == 'FAULT' or self.record or
                getattr(self, 'test_items', {})):
            response.message = 'change item mode only after clearing/reconciling and resetting the test'
            return response
        self.two_item_mode = bool(request.data)
        self.test_items = {}
        self.active_item = 0
        response.success = True
        response.message = 'two-item floor-only test' if request.data else 'single-item test'
        self.publish_status()
        return response

    def _other_pallet_items(self):
        return [deepcopy(v['record']) for k, v in self.test_items.items()
                if k != self.active_item and v['location'] == 'pallet']

    def _virtual_size(self, dimensions):
        return tuple(Item(FLB=Point3D(0, 0, 0), Dim=self.loader._coerce_dimension(dimensions),
                          buffer_space=self.loader.clearance_mm,
                          clearance_mode=self.loader.clearance_mode).Virtual_Dim.raw())

    def _floor_filter(self):
        others = self._other_pallet_items()
        clearance, mode = self.loader.clearance_mm, self.loader.clearance_mode
        def allowed(p):
            size = Item(FLB=p.flb, Dim=p.item_dim, buffer_space=clearance,
                        clearance_mode=mode).Virtual_Dim.raw()
            return p.flb.z == 0 and all(disjoint_xy((p.flb.x, p.flb.y, p.flb.z), size, r) for r in others)
        return allowed

    def _floor_available(self):
        others = self._other_pallet_items()
        dx, dy, dz = self.record['size_mm']
        if dz > self.container[2]:
            return False
        for dimensions in ((dx, dy, dz), (dy, dx, dz)):
            size = self._virtual_size(dimensions)
            xs, ys = {0., self.container[0]-size[0]}, {0., self.container[1]-size[1]}
            for r in others:
                xs.update((r['virtual_corner_mm'][0]-size[0], r['virtual_corner_mm'][0]+r['virtual_size_mm'][0]))
                ys.update((r['virtual_corner_mm'][1]-size[1], r['virtual_corner_mm'][1]+r['virtual_size_mm'][1]))
            for x in xs:
                for y in ys:
                    if (0 <= x <= self.container[0]-size[0] and 0 <= y <= self.container[1]-size[1]
                            and all(disjoint_xy((x, y, 0), size, r) for r in others)
                            and (self.step != 'repack' or
                                 [x, y, 0] != self.record['virtual_corner_mm'])):
                        return True
        return False

    def _two_choices(self):
        self._sync_item()
        saved = (self.active_item, self.location, self.record, self.test_slot, self.step)
        choices = {}
        free = sorted(s['slot'] for s in self.status['staging'].get('slots', [])
                      if type(s.get('slot')) is int and 0 <= s['slot'] < 6 and s.get('occupied') is False)
        try:
            if len(self.test_items) < 2:
                key = next(k for k in (0, 1) if k not in self.test_items)
                self._select_item(key)
                for slot in free:
                    if not self.check_preconditions('pack_new', slot):
                        choices.setdefault('pack_new', []).append((key, slot))
                return choices
            for key in sorted(self.test_items):
                self._select_item(key)
                steps = ('pack_slot',) if self.location == 'slot' else ('unpack', 'repack')
                for step in steps:
                    if step == 'repack' and self.random_pack_unpack_only:
                        continue
                    self.step = step
                    if step in ('pack_slot', 'repack') and not self._floor_available():
                        continue
                    for slot in (free if step == 'unpack' else [self.test_slot]):
                        if not self.check_preconditions(step, slot):
                            choices.setdefault(step, []).append((key, slot))
            return choices
        finally:
            self.active_item, self.location, self.record, self.test_slot, self.step = saved
