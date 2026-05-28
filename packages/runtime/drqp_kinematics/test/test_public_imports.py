import importlib


def test_public_imports_resolve():
    geometry = importlib.import_module('drqp_kinematics.geometry')
    models = importlib.import_module('drqp_kinematics.models')

    assert geometry.Point3D is not None
    assert models.HexapodModel is not None