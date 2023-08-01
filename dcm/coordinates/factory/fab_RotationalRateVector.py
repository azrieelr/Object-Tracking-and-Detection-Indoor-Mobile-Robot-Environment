import sympy as sp


from coordinates import RotationalRateVector
from coordinates.factory.fab_DirectionCosineMatrix import *



# equations

earth_const = {
    'semiMajorAxis': {
        'symbol': sp.Symbol('a_e'),
        'value': 6378137.0,
    },
    'linearExcentricity': {
        'symbol': sp.Symbol('e_e'),
        'value': 8.1819190842622e-2,
    },
    'spinRate': {
        'symbol': sp.Symbol('ω_e'),
        'value': 7.292115e-5,
    },
}

earth_var = {
    'lat': {
        'symbol': sp.Symbol('ϕ'),
        'value': sp.Symbol('ϕ'),
    },
    'long': {
        'symbol': sp.Symbol('λ'),
        'value': sp.Symbol('λ'),
    },
    'ellipsoidHeight': {
        'symbol': sp.Symbol('h_e'),
        'value': sp.Symbol('h_e'),
    },
    'radiiMeridian': {
        'symbol': sp.Symbol('M_e'),
        'value': sp.Symbol('M_e'),
    },
    'radiiPrimeVertical': {
        'symbol': sp.Symbol('N_e'),
        'value': sp.Symbol('N_e'),
    },
}
# set earth_var cross reference values
earth_var['radiiMeridian']['value'] = earth_const['semiMajorAxis']['value'] / sp.sqrt(1 - earth_const['linearExcentricity']['value']**2 * sp.sin(earth_var['lat']['value'])**2)
earth_var['radiiPrimeVertical']['value'] = earth_const['semiMajorAxis']['value'] * (1 - earth_const['linearExcentricity']['value']**2) / sp.sqrt((1 - earth_const['linearExcentricity']['value']**2 * sp.sin(earth_var['lat']['value'])**2)**3)


ned_var = {
    'velocityEasting': {
        'symbol': sp.Symbol('v_E'),
        'value': sp.Symbol('v_E'),
    },
    'velocityNorthing': {
        'symbol': sp.Symbol('v_N'),
        'value': sp.Symbol('v_N'),
    },
    'velocityDown': {
        'symbol': sp.Symbol('v_D'),
        'value': sp.Symbol('v_D'),
    },
}


# W_ie^e
def createRRV_InertialOfEarth_projEarth(get='symbol') -> RotationalRateVector:
    if get not in ['symbol', 'value']:
        raise ValueError("Invalid get. Must be 'symbol' or 'value'.")
    return RotationalRateVector(
        rotRateVec = sp.Matrix([0, 0, earth_const['spinRate'][get]]),
        wrtName= 'Inertial',
        ofName= 'Earth',
        projectionName= 'Earth'
    )

# W_ie^n
def createRRV_InertialOfEarth_projNavigation(get='symbol') -> RotationalRateVector:
    if get not in ['symbol', 'value']:
        raise ValueError("Invalid get. Must be 'symbol' or 'value'.")
    rrv = createRRV_InertialOfEarth_projEarth(get)
    rrv.setProjection(dcmProjection = createDCM_Earth2Navigation(), projectionName = 'Navigation')
    return rrv

# W_en^n
def createRRV_EarthOfNavigation_projNavigation(get='symbol') -> RotationalRateVector:
    if get not in ['symbol', 'value']:
        raise ValueError("Invalid get. Must be 'symbol' or 'value'.")
    return RotationalRateVector(
        rotRateVec = sp.Matrix([
            (ned_var['velocityEasting'][get] / (earth_var['radiiPrimeVertical'][get] + earth_var['ellipsoidHeight'][get])),
            (-ned_var['velocityNorthing'][get] / (earth_var['radiiMeridian'][get] + earth_var['ellipsoidHeight'][get])),
            (-ned_var['velocityEasting'][get] * sp.tan(earth_var['lat'][get]) / (earth_var['radiiMeridian'][get] + earth_var['ellipsoidHeight'][get]))
        ]),
        wrtName= 'Earth',
        ofName= 'Navigation',
        projectionName= 'Navigation'
    )

# W_in^n
def createRRV_InertialOfNavigation_projNavigation(get='symbol') -> RotationalRateVector:
    if get not in ['symbol', 'value']:
        raise ValueError("Invalid get. Must be 'symbol' or 'value'.")
    return createRRV_InertialOfEarth_projNavigation(get) + createRRV_EarthOfNavigation_projNavigation(get)