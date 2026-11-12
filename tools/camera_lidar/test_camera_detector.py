from camera_detector import parse_classes, pick_device, resolve_model

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
# Klassenliste: Liste wie bisher oder kommagetrennt (rollator.env), Leerzeichen in Namen bleiben erhalten
assert parse_classes(None) is None and parse_classes([]) is None and parse_classes([' , ']) is None
assert parse_classes(['person', 'chair']) == ['person', 'chair']
assert parse_classes(['person,dining table, chair']) == ['person', 'dining table', 'chair']
assert parse_classes(['dining table']) == ['dining table']
print('test_camera_detector ok')
