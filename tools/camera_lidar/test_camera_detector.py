from camera_detector import pick_device, resolve_model

# Geraetewahl
assert pick_device('auto', True) == 'cuda:0'
assert pick_device('auto', False) == 'cpu'
assert pick_device(None, False) == 'cpu'
assert pick_device('cpu', True) == 'cpu'
assert pick_device('cuda:0', True) == 'cuda:0'
assert pick_device('0', True) == 'cuda:0'
assert pick_device(' CUDA:0 ', True) == 'cuda:0'
for bad, cuda in (('cuda:0', False), ('0', False), ('gpu', True)):
    try:
        pick_device(bad, cuda)
        raise AssertionError(f'{bad} haette ValueError geben muessen')
    except ValueError:
        pass

# Modellpfad: neben dem Skript bevorzugt, sonst unveraendert; Pfade bleiben Pfade
assert resolve_model('yolo11n.pt', here='/x', exists=lambda p: p == '/x/yolo11n.pt') == '/x/yolo11n.pt'
assert resolve_model('yolo11n.engine', here='/x', exists=lambda p: False) == 'yolo11n.engine'
assert resolve_model('/abs/m.engine', here='/x', exists=lambda p: True) == '/abs/m.engine'
assert resolve_model('sub/m.pt', here='/x', exists=lambda p: True) == 'sub/m.pt'
print('test_camera_detector ok')
