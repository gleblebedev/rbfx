/*
 * This source file is part of RmlUi, the HTML/CSS Interface Middleware
 *
 * For the latest information, see http://github.com/mikke89/RmlUi
 *
 * Copyright (c) 2008-2010 CodePoint Ltd, Shift Technology Ltd
 * Copyright (c) 2019 The RmlUi Team, and contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "../../Include/RmlUi/Core/StyleSheetContainer.h"
#include "../../Include/RmlUi/Core/PropertyDictionary.h"
#include "../../Include/RmlUi/Core/Profiling.h"
#include "../../Include/RmlUi/Core/StyleSheet.h"
#include "ComputeProperty.h"
#include "StyleSheetParser.h"
#include "Utilities.h"

namespace Rml {

StyleSheetContainer::StyleSheetContainer()
{
}

StyleSheetContainer::~StyleSheetContainer()
{
}

bool StyleSheetContainer::LoadStyleSheetContainer(Stream* stream, int begin_line_number)
{
	StyleSheetParser parser;
	int rule_count = parser.Parse(media_blocks, stream, begin_line_number);
	return rule_count >= 0;
}

bool StyleSheetContainer::UpdateCompiledStyleSheet(float dp_ratio, Vector2f vp_dimensions)
{
	RMLUI_ZoneScoped;

	Vector<int> new_active_media_block_indices;

	const float font_size = DefaultComputedValues.font_size;

	for (int media_block_index = 0; media_block_index < (int)media_blocks.size(); media_block_index++)
	{
		const MediaBlock& media_block = media_blocks[media_block_index];
		bool all_match = true;
		for (const auto& property : media_block.properties.GetProperties())
		{
			const MediaQueryId id = static_cast<MediaQueryId>(property.first);

			switch (id)
			{
			case MediaQueryId::Width:
				if (vp_dimensions.x != ComputeLength(&property.second, font_size, font_size, dp_ratio, vp_dimensions))
					all_match = false;
				break;
			case MediaQueryId::MinWidth:
				if (vp_dimensions.x < ComputeLength(&property.second, font_size, font_size, dp_ratio, vp_dimensions))
					all_match = false;
				break;
			case MediaQueryId::MaxWidth:
				if (vp_dimensions.x > ComputeLength(&property.second, font_size, font_size, dp_ratio, vp_dimensions))
					all_match = false;
				break;
			case MediaQueryId::Height:
				if (vp_dimensions.y != ComputeLength(&property.second, font_size, font_size, dp_ratio, vp_dimensions))
					all_match = false;
				break;
			case MediaQueryId::MinHeight:
				if (vp_dimensions.y < ComputeLength(&property.second, font_size, font_size, dp_ratio, vp_dimensions))
					all_match = false;
				break;
			case MediaQueryId::MaxHeight:
				if (vp_dimensions.y > ComputeLength(&property.second, font_size, font_size, dp_ratio, vp_dimensions))
					all_match = false;
				break;
			case MediaQueryId::AspectRatio:
				if ((vp_dimensions.x / vp_dimensions.y) != property.second.Get<float>())
					all_match = false;
				break;
			case MediaQueryId::MinAspectRatio:
				if ((vp_dimensions.x / vp_dimensions.y) < property.second.Get<float>())
					all_match = false;
				break;
			case MediaQueryId::MaxAspectRatio:
				if ((vp_dimensions.x / vp_dimensions.y) > property.second.Get<float>())
					all_match = false;
				break;
			case MediaQueryId::Resolution:
				if (dp_ratio != property.second.Get<float>())
					all_match = false;
				break;
			case MediaQueryId::MinResolution:
				if (dp_ratio < property.second.Get<float>())
					all_match = false;
				break;
			case MediaQueryId::MaxResolution:
				if (dp_ratio > property.second.Get<float>())
					all_match = false;
				break;
			case MediaQueryId::Orientation:
				// Landscape (x > y) = 0 
				// Portrait (x <= y) = 1
				if ((vp_dimensions.x <= vp_dimensions.y) != property.second.Get<bool>())
					all_match = false;
				break;
				// Invalid properties
			case MediaQueryId::Invalid:
			case MediaQueryId::NumDefinedIds:
				break;
			}

			if (!all_match)
				break;
		}

		if (all_match)
			new_active_media_block_indices.push_back(media_block_index);
	}

	const bool style_sheet_changed = (new_active_media_block_indices != active_media_block_indices || !compiled_style_sheet);

	if (style_sheet_changed)
	{
		StyleSheet* first_sheet = nullptr;
		UniquePtr<StyleSheet> new_sheet;

		for (int index : new_active_media_block_indices)
		{
			MediaBlock& media_block = media_blocks[index];
			if (!first_sheet)
				first_sheet = media_block.stylesheet.get();
			else
				new_sheet = (new_sheet ? new_sheet.get() : first_sheet)->CombineStyleSheet(*media_block.stylesheet);
		}

		if (!first_sheet)
		{
			new_sheet.reset(new StyleSheet);
			first_sheet = new_sheet.get();
		}

		compiled_style_sheet = (new_sheet ? new_sheet.get() : first_sheet);
		combined_compiled_style_sheet = std::move(new_sheet);

		compiled_style_sheet->BuildNodeIndex();
		compiled_style_sheet->OptimizeNodeProperties();
	}

	active_media_block_indices = std::move(new_active_media_block_indices);

	return style_sheet_changed;
}

StyleSheet* StyleSheetContainer::GetCompiledStyleSheet()
{
	return compiled_style_sheet;
}

SharedPtr<StyleSheetContainer> StyleSheetContainer::CombineStyleSheetContainer(const StyleSheetContainer& container) const
{
	RMLUI_ZoneScoped;

	SharedPtr<StyleSheetContainer> new_sheet = MakeShared<StyleSheetContainer>();

	for (const MediaBlock& media_block : media_blocks)
	{
		new_sheet->media_blocks.emplace_back(media_block.properties, media_block.stylesheet->Clone());
	}

	new_sheet->MergeStyleSheetContainer(container);

	return new_sheet;
}

void StyleSheetContainer::MergeStyleSheetContainer(const StyleSheetContainer& container)
{
	RMLUI_ZoneScoped;

	// Style sheet container must not be merged after it's been compiled. This will invalidate references to the compiled style sheet.
	RMLUI_ASSERT(!compiled_style_sheet);

	for (const MediaBlock& block_other : container.media_blocks)
	{
		bool block_found = false;
		for (MediaBlock& block_local : media_blocks)
		{
			if (block_other.properties.GetProperties() == block_local.properties.GetProperties())
			{
				block_local.stylesheet = block_local.stylesheet->CombineStyleSheet(*block_other.stylesheet);
				block_found = true;
				break;
			}
		}

		if (!block_found)
		{
			media_blocks.emplace_back(block_other.properties, block_other.stylesheet->Clone());
		}
	}
}

void StyleSheetContainer::OptimizeNodeProperties()
{
	for (MediaBlock& block : media_blocks)
		block.stylesheet->OptimizeNodeProperties();
}

} // namespace Rml
