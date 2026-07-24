from pathlib import Path
from xml.dom import minidom

import pytest
import xacro


PROFILE_XACRO = (
    Path(__file__).resolve().parents[1] / "description" / "robstride_motor_profiles.xacro"
)
PROFILE_MACROS = [
    "robstride_rs00_params",
    "robstride_rs01_params",
    "robstride_rs02_params",
    "robstride_rs03_params",
    "robstride_rs04_params",
    "robstride_rs05_params",
    "robstride_rs06_params",
    "robstride_edulite05_params",
]


def process_profile(macro: str, arguments: str) -> None:
    document = minidom.parseString(
        f"""
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gain_test">
          <xacro:include filename="{PROFILE_XACRO}"/>
          <xacro:{macro} can_id="1" {arguments}/>
        </robot>
        """
    )
    xacro.process_doc(document)


@pytest.mark.parametrize("macro", PROFILE_MACROS)
def test_profile_requires_kp(macro: str) -> None:
    with pytest.raises(xacro.XacroException):
        process_profile(macro, 'kd="1.0"')


@pytest.mark.parametrize("macro", PROFILE_MACROS)
def test_profile_requires_kd(macro: str) -> None:
    with pytest.raises(xacro.XacroException):
        process_profile(macro, 'kp="30.0"')


@pytest.mark.parametrize("macro", PROFILE_MACROS)
def test_profile_accepts_explicit_gains(macro: str) -> None:
    process_profile(macro, 'kp="30.0" kd="1.0"')
