import React from "react"
import { Route, Routes, Navigate } from "react-router-dom"
import { PATHS } from "./components/vars"
import * as defaults from "./templates"
import { VirtualHexapod } from "hexapod-kinematics-library"
import {
    InverseKinematicsPage,
    WalkingGaitsPage,
    ForwardKinematicsPage,
    LegPatternPage,
    LandingPage,
} from "./components/pages"

const Page = ({ pageComponent }) => (
    <Routes>
        <Route path={PATHS.landingPage.path} exact component={pageComponent(LandingPage)} />
        <Route path={PATHS.legPatterns.path} component={pageComponent(LegPatternPage)} />
        <Route path={PATHS.forwardKinematics.path} component={pageComponent(ForwardKinematicsPage)} />
        <Route path={PATHS.inverseKinematics.path} component={pageComponent(InverseKinematicsPage)} />
        <Route path={PATHS.walkingGaits.path} component={pageComponent(WalkingGaitsPage)} />
        <Route path="*" element={<Navigate to={PATHS.landingPage.path} />} />
    </Routes>
)

const updateHexapod = (updateType, newParam, oldHexapod) => {
    if (updateType === "default") {
        return new VirtualHexapod(defaults.DEFAULT_DIMENSIONS, defaults.DEFAULT_POSE)
    }

    let hexapod = null
    const { pose, dimensions } = oldHexapod

    if (updateType === "pose") {
        hexapod = new VirtualHexapod(dimensions, newParam.pose)
    }

    if (updateType === "dimensions") {
        hexapod = new VirtualHexapod(newParam.dimensions, pose)
    }

    if (updateType === "hexapod") {
        hexapod = newParam.hexapod
    }

    if (!hexapod || !hexapod.foundSolution) {
        return oldHexapod
    }

    return hexapod
}

export { Page, updateHexapod }
