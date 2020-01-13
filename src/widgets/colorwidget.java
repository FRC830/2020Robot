package edu.wpi.first.shuffleboard.plugin.base.widget;

import edu.wpi.first.shuffleboard.api.prefs.Group;
import edu.wpi.first.shuffleboard.api.prefs.Setting;
import edu.wpi.first.shuffleboard.api.widget.Description;
import edu.wpi.first.shuffleboard.api.widget.ParametrizedController;
import edu.wpi.first.shuffleboard.api.widget.SimpleAnnotatedWidget;

import com.google.common.collect.ImmutableList;

import java.util.List;

import javafx.beans.binding.Bindings;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
/**
 * Widget
 */
@Description(
    name = "Color Picker",
    dataTypes = Color.class)
@ParametrizedController("BooleanBoxWidget.fxml")
// https://docs.oracle.com/javafx/2/ui_controls/color-picker.htm
// https://github.com/wpilibsuite/shuffleboard/blob/6dabb35bb443e043a40b78782e17519ad80ac033/plugins/base/src/main/resources/edu/wpi/first/shuffleboard/plugin/base/widget/BooleanBoxWidget.fxml
// https://github.com/wpilibsuite/shuffleboard/blob/6dabb35bb443e043a40b78782e17519ad80ac033/plugins/base/src/main/resources/edu/wpi/first/shuffleboard/plugin/base/widget/Command.fxml

public class colorwidget extends SimpleAnnotatedWidget {
    @FXML
    private Pane root;
  
    private final Property<Color> color = new SimpleObjectProperty<>(this, "color", Color.LAWNGREEN);
    @FXML
    private ColorPicker colorPicker;
    @FXML
    private void initialize() {
        fillBackground(color);
        colorPicker.setOnAction(new EventHandler() {
            public void handle(Event t) {
                fillBackground(colorPicker.getValue());               
            }
        });
    }
  
    @Override
    public List<Group> getSettings() {
      return ImmutableList.of();
    }
  
    @Override
    public Pane getView() {
      return root;
    }
  
    private Color getColor() {
      final Boolean data = getData();
      if (data == null) {
        return Color.BLACK;
      }
  
        return color.getValue();
    }
  
    private Background fillBackground(Color color) {
      return new Background(new BackgroundFill(color, null, null));
    }
  
    public void updateColor(Color newColor) {
      fillBackground(newColor);
    }
}