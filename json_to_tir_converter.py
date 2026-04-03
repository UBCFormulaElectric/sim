"""
Convert Tire JSON data to TIR (Magic Formula) format.

This script reads a JSON file containing tire parameters and converts it
to the TIR format used by Magic Formula tire models.
"""

import json
import sys
from datetime import datetime
from pathlib import Path


def read_json_file(json_path):
    """Read and parse the JSON tire data file."""
    with open(json_path, 'r') as f:
        return json.load(f)


def extract_section_data(data, section_name):
    """Extract a specific section from the JSON data."""
    return data.get(section_name, {})


def format_tir_line(parameter_name, value, description=""):
    """Format a TIR parameter line."""
    # Convert Python floats to string with appropriate precision
    if isinstance(value, float):
        # Use scientific notation for very small/large values or format normally
        if abs(value) < 1e-6 or abs(value) > 1e6:
            value_str = f"{value:.10e}"
        else:
            value_str = f"{value:g}"
    else:
        value_str = str(value)
    
    # Format: PARAMETER = value   $description
    line = f"{parameter_name:<30}= {value_str:<20}"
    if description:
        line += f"${description}"
    return line


def create_mdi_header(tire_model_name="Hoosier_Tire"):
    """Create the MDI_HEADER section."""
    lines = [
        "[MDI_HEADER]",
        f"FILE_TYPE                = 'tir'",
        f"FILE_VERSION             = 3.0",
        f"FILE_FORMAT              = 'ASCII'",
        f"!",
        f"! Converted from JSON format",
        f"! Date: {datetime.now().strftime('%d-%b-%Y %H:%M:%S')}",
        f"!",
        f"! Model name: {tire_model_name}",
        f"!",
    ]
    return lines


def create_units_section():
    """Create the UNITS section."""
    lines = [
        "$" + "-" * 64 + "units",
        "[UNITS]",
        "LENGTH                   = 'meter'",
        "FORCE                    = 'newton'",
        "ANGLE                    = 'radians'",
        "MASS                     = 'kg'",
        "TIME                     = 'second'",
    ]
    return lines


def create_model_section(data):
    """Create the MODEL section."""
    basic = extract_section_data(data, 'basic')
    
    lines = [
        "$" + "-" * 64 + "model",
        "[MODEL]",
        "FITTYP                   = 62                 $Magic Formula Version number",
        # "TYRESIDE                 = 'RIGHT'",
        f"LONGVL                   = {basic.get('LONGVL', 16.7):<17}$Nominal speed",
        "PROPERTY_FILE_FORMAT     = 'MF-TYRE'",
        # "FUNCTION_NAME            = ''",
        # "N_TIRE_STATES            = 0",
        # "USE_MODE                 = 14                 $Tire use mode switch (ADAMS only)",
        "PLYSTEER                 = 'false'             $Plysteer",
        "TURNSLIP                 = 'false'             $Turnslip",
        "VXLOW                    = 1                  $Lower boundary of slip calculation",
    ]
    return lines


def create_dimension_section(data):
    """Create the DIMENSION section."""
    basic = extract_section_data(data, 'basic')
    
    lines = [
        "$" + "-" * 64 + "dimension",
        "[DIMENSION]",
        f"UNLOADED_RADIUS          = {basic.get('R0', 0.2286):<17}$Free tire radius",
        "WIDTH                    = 0.1524             $Nominal section width of the tire",
        "ASPECT_RATIO             = 0.67               $Nominal aspect ratio",
        "RIM_RADIUS               = 0.127              $Nominal rim radius",
        "RIM_WIDTH                = 0.1778             $Rim width",
    ]
    return lines


def create_operating_conditions_section(data):
    """Create the OPERATING_CONDITIONS section."""
    basic = extract_section_data(data, 'basic')
    
    lines = [
        "$" + "-" * 64 + "operating_conditions",
        "[OPERATING_CONDITIONS]",
        f"INFLPRES                 = {basic.get('P0', 82585.4):<17}$Tire inflation pressure",
        f"NOMPRES                  = {basic.get('P0', 82585.4):<17}$Nominal tire inflation pressure",
    ]
    return lines


def create_inertia_section():
    """Create the INERTIA section."""
    lines = [
        "$" + "-" * 64 + "inertia",
        "[INERTIA]",
        "MASS                     = 9.3                $Tire Mass",
        "IXX                      = 0.391              $Tire diametral moment of inertia",
        "IYY                      = 0.736              $Tire polar moment of inertia",
    ]
    return lines


def create_vertical_section(data):
    """Create the VERTICAL section."""
    basic = extract_section_data(data, 'basic')
    loaded_radius = extract_section_data(data, 'loaded_radius')
    eff_rolling = extract_section_data(data, 'effective_rolling_radius')
    
    lines = [
        "$" + "-" * 64 + "vertical",
        "[VERTICAL]",
        f"FNOMIN                   = {basic.get('FZ0', 739.5):<17}$Nominal wheel load",
        f"VERTICAL_STIFFNESS       = {loaded_radius.get('VERTICAL_STIFFNESS', 209651):<17}$Tire vertical stiffness",
        "VERTICAL_DAMPING         = 50                 $Tire vertical damping",
        "MC_CONTOUR_A             = 0                  $Motorcycle contour ellipse A",
        "MC_CONTOUR_B             = 0                  $Motorcycle contour ellipse B",
        f"BREFF                    = {eff_rolling.get('BREFF', 8.386):<17}$Low load stiffness effective rolling radius",
        f"DREFF                    = {eff_rolling.get('DREFF', 0.25826):<17}$Peak value of effective rolling radius",
        f"FREFF                    = {eff_rolling.get('FREFF', 0.07394):<17}$High load stiffness effective rolling radius",
        f"Q_RE0                    = {loaded_radius.get('QRE0', 0.9974):<17}$Ratio of free tire radius with nominal tire radius",
        f"Q_V1                     = {loaded_radius.get('QV1', 0.00077):<17}$Tire radius increase with speed",
        f"Q_V2                     = {loaded_radius.get('QV2', 0.04667):<17}$Vertical stiffness increase with speed",
        f"Q_FZ2                    = {loaded_radius.get('QFZ2', 15.4):<17}$Quadratic term in load vs. deflection",
        f"Q_FCX                    = {loaded_radius.get('QFCX', 0):<17}$Longitudinal force influence on vertical stiffness",
        f"Q_FCY                    = {loaded_radius.get('QFCY', 0):<17}$Lateral force influence on vertical stiffness",
        f"Q_FCY2                   = {loaded_radius.get('QFCY2', 0):<17}$Explicit load dependency for including the lateral force influence on vertical stiffness",
        f"Q_CAM                    = {loaded_radius.get('QCAM', 0):<17}$Stiffness reduction due to camber",
        f"Q_CAM1                   = {loaded_radius.get('QCAM2', 0):<17}$Linear load dependent camber angle effect on vertical stiffness",
        f"Q_CAM2                   = {loaded_radius.get('QCAM3', 0):<17}$Quadratic load dependent camber angle effect on vertical stiffness",
        f"Q_CAM3                   = {loaded_radius.get('QFYS', 0):<17}$Linear reduction of stiffness with load and camber angle",
        f"Q_FYS1                   = {loaded_radius.get('QFYS', 0):<17}$Constant camber and slip angle effect on vertical stiffness",
        f"Q_FYS2                   = {loaded_radius.get('QFYS2', 0):<17}$Linear camber and slip angle effect on vertical stiffness",
        f"Q_FYS3                   = {loaded_radius.get('QFYS3', 0):<17}$Quadratic camber and slip angle effect on vertical stiffness",
        f"PFZ1                     = {loaded_radius.get('PFZ1', 0.7098):<17}$Pressure effect on vertical stiffness",
        "BOTTOM_OFFST             = 0.01               $Distance to rim when bottoming starts to occur",
        "BOTTOM_STIFF             = 2000000            $Vertical stiffness of bottomed tire",
    ]
    return lines


def create_structural_section(data):
    """Create the STRUCTURAL section."""
    transient = extract_section_data(data, 'transient')
    
    lines = [
        "$" + "-" * 64 + "structural",
        "[STRUCTURAL]",
        f"LONGITUDINAL_STIFFNESS   = {transient.get('LONGITUDINAL_STIFFNESS', 358066):<17}$Tire overall longitudinal stiffness",
        f"LATERAL_STIFFNESS        = {transient.get('LATERAL_STIFFNESS', 102673):<17}$Tire overall lateral stiffness",
        "YAW_STIFFNESS            = 4795               $Tire overall yaw stiffness",
        "DAMP_VLOW                = 0.001              $Additional low speed damping (proportional to stiffness)",
        f"PCFX1                    = {transient.get('PCFX1', 0.17504):<17}$Tire overall longitudinal stiffness vertical deflection dependency linear term",
        f"PCFX2                    = {transient.get('PCFX2', 0):<17}$Tire overall longitudinal stiffness vertical deflection dependency quadratic term",
        f"PCFX3                    = {transient.get('PCFX3', 0):<17}$Tire overall longitudinal stiffness pressure dependency",
        f"PCFY1                    = {transient.get('PCFY1', 0.16365):<17}$Tire overall lateral stiffness vertical deflection dependency linear term",
        f"PCFY2                    = {transient.get('PCFY2', 0):<17}$Tire overall lateral stiffness vertical deflection dependency quadratic term",
        f"PCFY3                    = {transient.get('PCFY3', 0.24993):<17}$Tire overall lateral stiffness pressure dependency",
        "PCMZ1                    = 0                  $Tire overall yaw stiffness pressure dependency",
    ]
    return lines


def create_contact_patch_section():
    """Create the CONTACT_PATCH section."""
    lines = [
        "$" + "-" * 64 + "contact_patch",
        "[CONTACT_PATCH]",
        "Q_RA1                    = 0.671              $Square root term in contact length equation",
        "Q_RA2                    = 0.733              $Linear term in contact length equation",
        "Q_RB1                    = 1.059              $Root term in contact width equation",
        "Q_RB2                    = -1.1878            $Linear term in contact width equation",
    ]
    return lines


def create_inflation_pressure_range_section(data):
    """Create the INFLATION_PRESSURE_RANGE section."""
    limits = extract_section_data(data, 'limits')
    
    lines = [
        "$" + "-" * 64 + "inflation_pressure_range",
        "[INFLATION_PRESSURE_RANGE]",
        f"PRESMIN                  = {limits.get('PRESMIN', 63):<17}$Minimum valid tire inflation pressure",
        f"PRESMAX                  = {limits.get('PRESMAX', 100):<17}$Maximum valid tire inflation pressure",
    ]
    return lines


def create_slip_ranges_section(data):
    """Create the slip range sections."""
    limits = extract_section_data(data, 'limits')
    
    lines = [
        "$" + "-" * 64 + "long_slip_range",
        "[LONG_SLIP_RANGE]",
        f"KPUMIN                   = {limits.get('KAPPAMIN', 0):<17}$Minimum valid wheel slip",
        f"KPUMAX                   = {limits.get('KAPPAMAX', 0):<17}$Maximum valid wheel slip",
        "$" + "-" * 64 + "slip_angle_range",
        "[SLIP_ANGLE_RANGE]",
        f"ALPMIN                   = {limits.get('ALPHAMIN', -0.2244):<17}$Minimum valid slip angle",
        f"ALPMAX                   = {limits.get('ALPHAMAX', 0.2225):<17}$Maximum valid slip angle",
        "$" + "-" * 64 + "inclination_angle_range",
        "[INCLINATION_ANGLE_RANGE]",
        f"CAMMIN                   = {limits.get('GAMMAMIN', -0.0843):<17}$Minimum valid camber angle",
        f"CAMMAX                   = {limits.get('GAMMAMAX', 0.0849):<17}$Maximum valid camber angle",
        "$" + "-" * 64 + "vertical_force_range",
        "[VERTICAL_FORCE_RANGE]",
        f"FZMIN                    = {limits.get('FZMIN', 100):<17}$Minimum allowed wheel load",
        f"FZMAX                    = {limits.get('FZMAX', 740):<17}$Maximum allowed wheel load",
    ]
    return lines


def create_scaling_coefficients_section(data):
    """Create the SCALING_COEFFICIENTS section."""
    scaling = extract_section_data(data, 'scaling')
    
    lines = [
        "$" + "-" * 64 + "scaling_coefficients",
        "[SCALING_COEFFICIENTS]",
    ]
    
    # Define the mapping of JSON keys to TIR parameter names
    scaling_params = [
        ('LFZ0', 'LFZO', 'Scale factor of nominal (rated) load'),
        ('LCX', 'LCX', 'Scale factor of Fx shape factor'),
        ('LMUX', 'LMUX', 'Scale factor of Fx peak friction coefficient'),
        ('LEX', 'LEX', 'Scale factor of Fx curvature factor'),
        ('LKX', 'LKX', 'Scale factor of Fx slip stiffness'),
        ('LHX', 'LHX', 'Scale factor of Fx horizontal shift'),
        ('LVX', 'LVX', 'Scale factor of Fx vertical shift'),
        ('LCY', 'LCY', 'Scale factor of Fy shape factor'),
        ('LMUY', 'LMUY', 'Scale factor of Fy peak friction coefficient'),
        ('LEY', 'LEY', 'Scale factor of Fy curvature factor'),
        ('LKY', 'LKY', 'Scale factor of Fy cornering stiffness'),
        ('LHY', 'LHY', 'Scale factor of Fy horizontal shift'),
        ('LVY', 'LVY', 'Scale factor of Fy vertical shift'),
        ('LTR', 'LTR', 'Scale factor of Peak of pneumatic trail'),
        ('LRES', 'LRES', 'Scale factor for offset of Mz residual torque'),
        ('LXAL', 'LXAL', 'Scale factor of alpha influence on Fx'),
        ('LYKA', 'LYKA', 'Scale factor of kappa influence on Fy'),
        ('LVYKA', 'LVYKA', 'Scale factor of kappa induced Fy'),
        ('LS', 'LS', 'Scale factor of Moment arm of Fx'),
        ('LKYC', 'LKYC', 'Scale factor of Fy camber stiffness'),
        ('LKZC', 'LKZC', 'Scale factor of Mz camber stiffness'),
        ('LVMX', 'LVMX', 'Scale factor of Mx vertical shift'),
        ('LMX', 'LMX', 'Scale factor of Mx overturning moment'),
        ('LMY', 'LMY', 'Scale factor of rolling resistance torque'),
        ('LMP', 'LMP', 'Scale factor of Mz parking torque'),
        ('LCZ', 'LCZ', 'Scale factor radial tire stiffness'),
        ('LMUV', 'LMUV', 'Scale factor with slip speed decaying friction'),
    ]
    
    for json_key, tir_key, desc in scaling_params:
        value = scaling.get(json_key, 1.0)
        lines.append(format_tir_line(tir_key, value, desc))
    
    return lines


def create_longitudinal_coefficients_section(data):
    """Create the LONGITUDINAL_COEFFICIENTS section."""
    longitudinal = extract_section_data(data, 'longitudinal')
    
    lines = [
        "$" + "-" * 64 + "longitudinal_coefficients",
        "[LONGITUDINAL_COEFFICIENTS]",
    ]
    
    # Map all longitudinal parameters
    params = [
        'PCX1', 'PDX1', 'PDX2', 'PDX3', 'PEX1', 'PEX2', 'PEX3', 'PEX4',
        'PKX1', 'PKX2', 'PKX3', 'PHX1', 'PHX2', 'PVX1', 'PVX2',
        'PPX1', 'PPX2', 'PPX3', 'PPX4',
        'RBX1', 'RBX2', 'RBX3', 'RCX1', 'REX1', 'REX2', 'RHX1'
    ]
    
    descriptions = {
        'PCX1': 'Shape factor Cfx for longitudinal force',
        'PDX1': 'Longitudinal friction Mux at Fznom',
        'PDX2': 'Variation of friction Mux with load',
        'PDX3': 'Variation of friction Mux with camber',
        # Add more as needed
    }
    
    for param in params:
        if param not in longitudinal:
            print(f"Missing longitudinal coefficient: {param}")
            lines.append(format_tir_line(param, 0, "MISSING VALUE"))
            continue
        value = longitudinal[param]
        lines.append(
            format_tir_line(param, value,
                            descriptions.get(param, ''))
        )
    
    return lines


def create_lateral_coefficients_section(data):
    """Create the LATERAL_COEFFICIENTS section."""
    lateral = extract_section_data(data, 'lateral')
    
    lines = [
        "$" + "-" * 64 + "lateral_coefficients",
        "[LATERAL_COEFFICIENTS]",
    ]
    
    params = [
        'PCY1', 'PDY1', 'PDY2', 'PDY3', 'PEY1', 'PEY2', 'PEY3', 'PEY4', 'PEY5',
        'PKY1', 'PKY2', 'PKY3', 'PKY4', 'PKY5', 'PKY6', 'PKY7',
        'PHY1', 'PHY2', 'PVY1', 'PVY2', 'PVY3', 'PVY4',
        'PPY1', 'PPY2', 'PPY3', 'PPY4', 'PPY5',
        'RBY1', 'RBY2', 'RBY3', 'RBY4',
        'RCY1', 'REY1', 'REY2', 'RHY1', 'RHY2',
        'RVY1', 'RVY2', 'RVY3', 'RVY4', 'RVY5', 'RVY6'
    ]
    
    for param in params:
        if param not in lateral:
            print(f"Missing lateral coefficient: {param}")
            lines.append(format_tir_line(param, 0, "MISSING VALUE"))
            continue
        value = lateral[param]
        lines.append(format_tir_line(param, value))
    
    return lines


def create_aligning_coefficients_section(data):
    """Create the ALIGNING_COEFFICIENTS section."""
    aligning = extract_section_data(data, 'aligning')
    
    lines = [
        "$" + "-" * 64 + "aligning_coefficients",
        "[ALIGNING_COEFFICIENTS]",
    ]
    
    params = [
        'QBZ1', 'QBZ2', 'QBZ3', 'QBZ4', 'QBZ5', 'QBZ6', 'QBZ9', 'QBZ10',
        'QCZ1',
        'QDZ1', 'QDZ2', 'QDZ3', 'QDZ4', 'QDZ6', 'QDZ7', 'QDZ8', 'QDZ9', 'QDZ10', 'QDZ11',
        'QEZ1', 'QEZ2', 'QEZ3', 'QEZ4', 'QEZ5',
        'QHZ1', 'QHZ2', 'QHZ3', 'QHZ4',
        'PPZ1', 'PPZ2',
        'SSZ1', 'SSZ2', 'SSZ3', 'SSZ4'
    ]
    
    for param in params:
        if param not in aligning:
            print(f"Missing aligning coefficient: {param}")
            lines.append(format_tir_line(param, 0, "MISSING VALUE"))
            continue
        value = aligning[param]
        lines.append(format_tir_line(param, value))
    
    return lines


def create_overturning_coefficients_section(data):
    """Create the OVERTURNING_COEFFICIENTS section."""
    overturning = extract_section_data(data, 'overturning')
    
    lines = [
        "$" + "-" * 64 + "overturning_coefficients",
        "[OVERTURNING_COEFFICIENTS]",
    ]
    
    params = [
        'QSX1', 'QSX2', 'QSX3', 'QSX4', 'QSX5', 'QSX6', 'QSX7', 'QSX8', 'QSX9', 'QSX10', 'QSX11', 'QSX12', 'QSX13', 'QSX14',
        'PPMX1'
    ]
    
    for param in params:
        if param not in overturning:
            print(f"Missing overturning coefficient: {param}")
            lines.append(format_tir_line(param, 0, "MISSING VALUE"))
            continue
        value = overturning[param]
        lines.append(format_tir_line(param, value))
    
    return lines


def create_rolling_coefficients_section(data):
    """Create the ROLLING_COEFFICIENTS section."""
    rolling = extract_section_data(data, 'rolling_resistance')
    
    lines = [
        "$" + "-" * 64 + "rolling_coefficients",
        "[ROLLING_COEFFICIENTS]",
    ]
    
    params = ['QSY1', 'QSY2', 'QSY3', 'QSY4', 'QSY5', 'QSY6', 'QSY7', 'QSY8']
    
    for param in params:
        assert param in rolling, f"Missing rolling resistance parameter: {param}"
        value = rolling[param]
        lines.append(format_tir_line(param, value))
    
    return lines


def create_turnslip_coefficients_section():
    """Create the TURNSLIP_COEFFICIENTS section."""
    lines = [
        "$" + "-" * 64 + "turnslip_coefficients",
        "[TURNSLIP_COEFFICIENTS]",
        "PDXP1                    = 0.4                $Peak Fx reduction due to spin parameter",
        "PDXP2                    = 0                  $Peak Fx reduction due to spin with varying load parameter",
        "PDXP3                    = 0                  $Peak Fx reduction due to spin with kappa parameter",
        "PKYP1                    = 1                  $Cornering stiffness reduction due to spin",
        "PDYP1                    = 0.4                $Peak Fy reduction due to spin parameter",
        "PDYP2                    = 0                  $Peak Fy reduction due to spin with varying load parameter",
        "PDYP3                    = 0                  $Peak Fy reduction due to spin with alpha parameter",
        "PDYP4                    = 0                  $Peak Fy reduction due to square root of spin parameter",
        "PHYP1                    = 1                  $Fy-alpha curve lateral shift limitation",
        "PHYP2                    = 0.15               $Fy-alpha curve maximum lateral shift parameter",
        "PHYP3                    = 0                  $Fy-alpha curve maximum lateral shift varying with load parameter",
        "PHYP4                    = -4                 $Fy-alpha curve maximum lateral shift parameter",
        "PECP1                    = 0.5                $Camber w.r.t. spin reduction factor parameter in camber stiffness",
        "PECP2                    = 0                  $Camber w.r.t. spin reduction factor varying with load parameter in camber stiffness",
        "QDTP1                    = 10                 $Pneumatic trail reduction factor due to turn slip parameter",
        "QCRP1                    = 0.2                $Turning moment at constant turning and zero forward speed parameter",
        "QCRP2                    = 0.1                $Turn slip moment (at alpha=90deg) parameter for increase with spin",
        "QBRP1                    = 0.1                $Residual (spin) torque reduction factor parameter due to side slip",
        "QDRP1                    = 1                  $Turn slip moment peak magnitude parameter",
        "QDRP2                    = 0                  $Turn slip moment curvature factor",
    ]
    return lines


def convert_json_to_tir(json_path, output_path=None):
    """
    Main conversion function.
    
    Args:
        json_path: Path to the input JSON file
        output_path: Path to the output TIR file (optional)
    
    Returns:
        Path to the generated TIR file
    """
    # Read JSON data
    print(f"Reading JSON file: {json_path}")
    data = read_json_file(json_path)
    
    # Generate output path if not provided
    if output_path is None:
        json_file = Path(json_path)
        output_path = json_file.parent / f"{json_file.stem}_converted.tir"
    
    # Build all sections
    print("Building TIR sections...")
    all_lines = []
    
    # Add all sections in order
    all_lines.extend(create_mdi_header())
    all_lines.append("")
    all_lines.extend(create_units_section())
    all_lines.append("")
    all_lines.extend(create_model_section(data))
    all_lines.append("")
    all_lines.extend(create_dimension_section(data))
    all_lines.append("")
    all_lines.extend(create_operating_conditions_section(data))
    all_lines.append("")
    all_lines.extend(create_inertia_section())
    all_lines.append("")
    all_lines.extend(create_vertical_section(data))
    all_lines.append("")
    all_lines.extend(create_structural_section(data))
    all_lines.append("")
    all_lines.extend(create_contact_patch_section())
    all_lines.append("")
    all_lines.extend(create_inflation_pressure_range_section(data))
    all_lines.append("")
    all_lines.extend(create_slip_ranges_section(data))
    all_lines.append("")
    all_lines.extend(create_scaling_coefficients_section(data))
    all_lines.append("")
    all_lines.extend(create_longitudinal_coefficients_section(data))
    all_lines.append("")
    all_lines.extend(create_lateral_coefficients_section(data))
    all_lines.append("")
    all_lines.extend(create_aligning_coefficients_section(data))
    all_lines.append("")
    all_lines.extend(create_overturning_coefficients_section(data))
    all_lines.append("")
    all_lines.extend(create_rolling_coefficients_section(data))
    all_lines.append("")
    all_lines.extend(create_turnslip_coefficients_section())
    
    # Write to file
    print(f"Writing TIR file: {output_path}")
    with open(output_path, 'w') as f:
        f.write('\n'.join(all_lines))
    
    print(f"✓ Conversion complete!")
    print(f"  Input:  {json_path}")
    print(f"  Output: {output_path}")
    
    return output_path


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python json_to_tir_converter.py <json_file> [output_tir_file]")
        print("\nExample:")
        print("  python json_to_tir_converter.py tire_data.json tire_output.tir")
        sys.exit(1)
    
    json_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    try:
        convert_json_to_tir(json_file, output_file)
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON format - {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
