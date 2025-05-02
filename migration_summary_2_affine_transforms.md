# Migration of 2_affine_transforms.md from Matplotlib to Plotly

## Changes Made

1. **Updated Imports**
   - Replaced matplotlib imports with Plotly equivalents
   - Updated all plotting module imports to use `plotting_plotly` instead of `plotting`

2. **Updated Plotting Functions**
   - Modified `plot_leg_with_points` calls to work with Plotly
   - Replaced matplotlib figure display with Plotly figure display
   - Converted subplot creation to use Plotly's `make_subplots`

3. **Updated 3D Visualization**
   - Replaced matplotlib 3D plotting with Plotly's 3D scatter and line plots
   - Updated camera view settings to use Plotly's camera configuration
   - Enhanced visual appearance with better default styling

4. **Updated DrQP Plotting**
   - Rewrote `plot_drqp` function to use Plotly's 3D plotting capabilities
   - Updated `update_drqp_plot` function to work with Plotly traces
   - Improved target visualization with Plotly's 3D scatter plots

5. **Updated Animation**
   - Replaced matplotlib animation with Plotly's interactive animation
   - Updated animation function to work with Plotly's update mechanism
   - Enhanced animation controls with Plotly's interactive sliders

## Benefits

1. **Better Interactivity**: Plotly provides native interactive features like zooming, panning, and hovering
2. **Improved Aesthetics**: Modern, clean visual style with better default colors and styling
3. **Better Performance**: More efficient rendering of complex 3D scenes and animations
4. **Web Compatibility**: Plotly visualizations work well in web environments like Jupyter notebooks
5. **Easier Customization**: More intuitive API for customizing plot appearance

## Files Modified
- `2_affine_transforms.md` - Updated all plotting code to use Plotly

## Files Used
- `plotting_plotly.py` - Plotly version of the plotting module
- `plotting_plotly_ik.py` - Plotly version of the inverse kinematics plotting functions
