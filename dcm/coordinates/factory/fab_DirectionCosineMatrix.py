import sympy as sp

from coordinates import DirectionCosineMatrix



# EARTH <---> NAVIGATION

def createDCM_Earth2Navigation(lat = sp.Symbol('ϕ'), long = sp.Symbol('λ'), type="radians") -> DirectionCosineMatrix:
    DCM_Earth2Navigation = DirectionCosineMatrix()
    if type == "radians":
        DCM_Earth2Navigation.compileRotMatrix('y', radians = -lat-(sp.pi/2), name = "lat")
        DCM_Earth2Navigation.compileRotMatrix('z', radians = long, name="long")
        return DCM_Earth2Navigation
    elif type == "degrees":
        DCM_Earth2Navigation.compileRotMatrix('y', degrees = -lat-90, name = "lat")
        DCM_Earth2Navigation.compileRotMatrix('z', degrees = long, name="long")
        return DCM_Earth2Navigation
    else:
        raise ValueError("Invalid type. Must be 'radians' or 'degrees'.")

def createDCM_Navigation2Earth(lat = sp.Symbol('ϕ'), long = sp.Symbol('λ'), type="radians") -> DirectionCosineMatrix:
    return createDCM_Earth2Navigation(lat = lat, long = long, type = type).T



# NAVIGATION <---> BODY

def createDCM_Navigation2Body(pitch = sp.Symbol("θ"), roll = sp.Symbol("φ"), yaw = sp.Symbol("ψ"), type="radians") -> DirectionCosineMatrix:
    DCM_Navigation2Body = DirectionCosineMatrix()
    if type == "radians":
        DCM_Navigation2Body.compileRotMatrix('x', radians = yaw, name = "yaw")
        DCM_Navigation2Body.compileRotMatrix('y', radians = roll, name = "roll")
        DCM_Navigation2Body.compileRotMatrix('z', radians = pitch, name = "pitch")
        return DCM_Navigation2Body
    elif type == "degrees":
        DCM_Navigation2Body.compileRotMatrix('x', degrees = pitch, name = "pitch")
        DCM_Navigation2Body.compileRotMatrix('y', degrees = roll, name = "roll")
        DCM_Navigation2Body.compileRotMatrix('z', degrees = yaw, name = "yaw")
        return DCM_Navigation2Body

def createDCM_Body2Navigation(pitch = sp.Symbol("θ"), roll = sp.Symbol("φ"), yaw = sp.Symbol("ψ"), type="radians") -> DirectionCosineMatrix:
    return createDCM_Navigation2Body(pitch = pitch, roll = roll, yaw = yaw, type = type).T

if __name__ == '__main__':
    cbn = createDCM_Body2Navigation()
    sp.pprint(cbn)

    cnb = cbn.T
    sp.pprint(cnb)