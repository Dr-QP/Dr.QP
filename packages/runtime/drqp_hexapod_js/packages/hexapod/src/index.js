import React, { Suspense } from "react"
import ReactDOM from 'react-dom/client';
import "./index.css"
import reportWebVitals from './reportWebVitals';

const App = React.lazy(() => import("./App"))

const container = document.getElementById('root')
const root = ReactDOM.createRoot(container);
root.render(
    <React.StrictMode>
        <Suspense fallback={<p>Mithi's Bare Minimum Hexapod Robot Simulator...</p>}>
            <App />
        </Suspense>
    </React.StrictMode>
)

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
